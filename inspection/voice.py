#!/usr/bin/env python3
"""Talk to the robot while it works (simulation).

    ../.venv/bin/python voice.py                  # live: type or dictate, the robot answers out loud
    ../.venv/bin/python voice.py --mic            # live: speak to the Mac's microphone (Whisper, offline)
    ../.venv/bin/python voice.py --speed 4        # simulation 4x faster than real time
    ../.venv/bin/python voice.py --mute --script "inspect every 10 cm up to 20 cm; +6 stop; +2 status; resume; come down; report"
    ../.venv/bin/python voice.py --selftest
    ../.venv/bin/python voice.py --mictest        # spoken orders -> text -> orders, no microphone needed

The robot climbs in MuJoCo in real time and listens the whole time, so "stop", "status" or a
new order work in the middle of a climb. It understands:

    climb to 40 cm / climb 20 cm / come down / come down to 10 cm
    inspect every 10 cm up to 50 cm        photo        stop / resume
    status        check your grip          report       quit

Speech out: macOS `say`. Speech in: typed lines, or `--mic` for the microphone (Whisper base.en
through faster-whisper; needs `pip install faster-whisper sounddevice` and downloads the 145 MB
model once). "Stop", "pause", "freeze" and "quit" also work while the robot is talking and cut
it off (its own voice is subtracted from the microphone first); other orders wait until it is
quiet. A bare Return always stops it. In a noisy room raise the threshold with `--mic-level 0.03`. Anything the rules
above don't cover goes to Claude (mission planner, or the manual assistant for questions) when
a valid ANTHROPIC_API_KEY is set; without one the robot says it did not understand.

In simulation the wall has no real cracks, so the camera is a stand-in: each 10 cm band of
the wall is given one photo from the held-out part of the crack dataset (data/cracks, seeded),
and the real crack CNN classifies it. Without the dataset, photos are saved but not checked.
"""
import contextlib
import io
import json
import queue
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import wave
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "sim"))

import crawl_gait  # noqa: E402
from env import FEET, ClimbEnv  # noqa: E402

MAX_CM = 100.0               # test wall, same limit as mission.py
STRIDE = 0.04                # m per gait cycle
BAND_CM = 10                 # the stand-in wall has one photo per band
LEG = {"fl": "front-left", "fr": "front-right", "hl": "hind-left", "hr": "hind-right"}
HELP = "Try: climb to 40 centimetres, inspect every 10 centimetres up to 50, photo, stop, resume, status, come down, report."


# -- understanding -----------------------------------------------------------
def cm(number, unit):
    return float(number) * (100 if unit and unit.startswith("m") and not unit.startswith("mm") else 1)


def parse(text):
    """Sentence -> (intent, arguments). Rules only; None means 'ask Claude or give up'."""
    t = text.lower().strip().rstrip(".!")
    nums = [cm(n, u) for n, u in re.findall(r"(\d+(?:\.\d+)?)\s*(centi\w*|cm|met\w*|m\b)?", t)]
    has = lambda *words: any(re.search(rf"\b{w}", t) for w in words)
    if has("quit", "exit", "bye", "goodbye"):
        return "quit", {}
    if has("why"):
        return "why", {}
    if has("stop", "freeze", "hold", "wait", "pause"):
        return "stop", {}
    if has("resume", "continue", "carry on", "go on", "keep going"):
        return "resume", {}
    if has("every") and len(nums) >= 2:
        return "inspect", {"step": nums[0], "top": nums[1]}
    if has("status", "where are", "how high", "how far", "position"):
        return "status", {}
    if has("grip", "pads", "self.?check"):
        return "grip", {}
    if has("report", "summary", "what did you find", "findings"):
        return "report", {}
    if has("come down", "go down", "descend", "climb down", "back down", "return", "come back", "home"):
        return "goto", {"cm": nums[0] if nums else 0.0, "relative": False}
    if has("climb", "go up", "up", "higher", "go to", "move"):
        if not nums:
            return None
        return "goto", {"cm": nums[0], "relative": not has("to") or has("another", "more")}
    if has("photo", "picture", "look", "inspect", "check"):
        return "photo", {}
    return None


def read_wav(path):
    with wave.open(str(path)) as fh:
        return np.frombuffer(fh.readframes(fh.getnframes()), np.int16).astype(np.float32) / 32768


# -- listening (microphone) --------------------------------------------------
RATE, BLOCK = 16000, 1600    # 16 kHz mono in 0.1 s blocks
MIC_FLOOR = 0.008            # quietest RMS counted as speech; raise it (--mic-level) in a noisy room
PROMPT = ("Orders for a climbing robot. Climb to 40 cm. Inspect every 10 cm up to 50 cm. Stop. Resume. "
          "Status. Check your grip. Photo. Come down. Report.")   # biases Whisper to these words and to digits


def utterances(blocks, level, speaking=lambda: False, end_quiet=7, longest=100, chunk=15, overlap=5):
    """Cut a stream of 0.1 s blocks into (audio, robot_was_speaking). A sentence starts when a
    block is louder than `level` and ends after 0.7 s of quiet. While the robot speaks there is
    no quiet, so the audio is cut every 1.5 s instead, with 0.5 s overlap so a word is not split."""
    from collections import deque
    before, heard, quiet, during = deque(maxlen=3), [], 0, False
    for x in blocks:
        talk = speaking()
        loud = float(np.sqrt(np.mean(x ** 2))) > level
        if not heard and not loud:
            before.append(x)                          # keep 0.3 s so the first syllable is not cut
            continue
        heard = heard or list(before)
        heard.append(x)
        during = during or talk
        quiet = 0 if loud else quiet + 1
        if quiet >= end_quiet or len(heard) >= (chunk if talk else longest):
            yield np.concatenate(heard), during
            heard = heard[-overlap:] if talk and quiet < end_quiet else []
            before.clear()
            quiet, during = 0, bool(heard)


def remove_own_voice(mic, said, taps=3200, early=64):
    """Subtract the robot's own sentence from what the microphone heard. `said` is the audio the
    robot is playing; the room changes it (delay, loudness, 200 ms of echo), so find the delay by
    correlation and fit that echo as a filter (Wiener solution). What is left is the person talking."""
    from scipy.linalg import solve_toeplitz
    from scipy.signal import fftconvolve
    if len(said) < taps or len(mic) < 2 * taps:
        return mic
    lag = int(np.argmax(np.abs(fftconvolve(mic, said[::-1])))) - (len(said) - 1)   # mic[n] ~ said[n - lag]
    first = early - lag - (taps - 1)                  # ref[j] = said[first + j]; mic[n] ~ sum h[k] ref[n + taps-1 - k]
    ref = np.zeros(len(mic) + taps - 1, np.float32)
    lo, hi = max(first, 0), min(first + len(ref), len(said))
    if hi <= lo:
        return mic
    ref[lo - first:hi - first] = said[lo:hi]
    auto = fftconvolve(ref, ref[::-1])[len(ref) - 1:len(ref) - 1 + taps]
    cross = fftconvolve(mic, ref[::-1])[len(mic) - 1:len(mic) - 1 + taps]
    auto[0] = auto[0] * 1.001 + 1e-6
    h = solve_toeplitz(auto, cross)
    return (mic - fftconvolve(ref, h)[taps - 1:taps - 1 + len(mic)]).astype(np.float32)


def loudest(audio):
    """RMS of the loudest 0.1 s. Not the average: one short word in 1.5 s of quiet must still count."""
    n = len(audio) // BLOCK * BLOCK
    return float(np.sqrt((audio[:n].reshape(-1, BLOCK) ** 2).mean(axis=1).max())) if n else 0.0


INTERRUPT = ("stop", "freeze", "pause", "quit")      # the only orders accepted over the robot's own voice


def barge_in(heard, robot_says):
    """While the robot talks the microphone also hears the robot. Accept only an interrupt word
    that the robot is not saying itself ('Stopped at 4 cm' must not stop it)."""
    own = re.findall(r"[a-z]+", robot_says.lower())
    for w in re.findall(r"[a-z]+", heard.lower()):
        for order in INTERRUPT:
            if w.startswith(order) and not any(o.startswith(order) for o in own):
                return order
    return None


def recogniser():
    """Whisper base.en on the CPU, offline after the first download (about 145 MB)."""
    from faster_whisper import WhisperModel
    model = WhisperModel("base.en", device="cpu", compute_type="int8")

    def to_text(audio):
        parts, _ = model.transcribe(audio, language="en", initial_prompt=PROMPT, vad_filter=True,
                                    condition_on_previous_text=False)
        return " ".join(p.text.strip() for p in parts if p.no_speech_prob < 0.6).strip()
    return to_text


def listen(inbox, voice, level=None):
    """Thread: microphone -> sentences -> text -> the same inbox the keyboard feeds."""
    import sounddevice as sd
    to_text = recogniser()
    last = [0.0]

    def robot_talking():                              # its sound reaches the microphone a little late
        if voice.speaking:
            last[0] = time.time()
        return time.time() - last[0] < 1.0

    while True:
        try:
            with sd.InputStream(samplerate=RATE, channels=1, dtype="float32", blocksize=BLOCK) as mic:
                read = lambda: mic.read(BLOCK)[0][:, 0].copy()
                rooms = []
                while level is None:                  # the quietest of 3 seconds, robot silent, sets the threshold
                    quiet = not robot_talking()
                    room = np.concatenate([read() for _ in range(10)])
                    if quiet and not robot_talking():
                        rooms.append(float(np.sqrt(np.mean(room ** 2))))
                    if len(rooms) == 3:
                        level = max(MIC_FLOOR, 3 * min(rooms))
                        print(f"(microphone on, speech level {level:.3f}; say stop at any time, "
                              "other orders when the robot is quiet)", flush=True)
                for audio, during in utterances((read() for _ in iter(int, 1)), level, robot_talking):
                    if during:                        # the robot is talking: take its voice out first
                        audio = remove_own_voice(audio, voice.audio)
                        if loudest(audio) < level:
                            continue                  # nothing left, so nobody spoke over it
                        over = to_text(audio)
                        text = barge_in(over, voice.now)
                        if over and not text:         # shown, so a missed "stop" can be diagnosed
                            print(f"(heard while speaking, ignored: {over})", flush=True)
                    else:
                        text = to_text(audio)
                    if text:
                        inbox.put(text)
        except sd.PortAudioError:                     # macOS restarts audio when playback begins or a device changes
            time.sleep(0.2)


def mictest():
    """Recogniser check without a microphone: `say` speaks three orders into a file, and the
    cutter, Whisper and the rules must turn them back into the right orders."""
    orders = [("climb to 40 centimetres", ("goto", {"cm": 40.0, "relative": False})),
              ("inspect every 10 centimetres up to 50", ("inspect", {"step": 10.0, "top": 50.0})),
              ("stop", ("stop", {})), ("check your grip", ("grip", {})), ("come down", ("goto", {"cm": 0.0, "relative": False}))]
    stream = [np.zeros(RATE, np.float32)]
    with tempfile.TemporaryDirectory() as tmp:
        for text, _ in orders:
            subprocess.run(["say", "-o", f"{tmp}/a.wav", "--data-format=LEI16@16000", text], check=True)
            with wave.open(f"{tmp}/a.wav") as fh:
                stream += [np.frombuffer(fh.readframes(fh.getnframes()), np.int16).astype(np.float32) / 32768,
                           np.zeros(RATE, np.float32)]
    stream = np.concatenate(stream)
    stream += np.random.default_rng(0).normal(0, 0.001, len(stream)).astype(np.float32)   # room hiss
    blocks = [stream[i:i + BLOCK] for i in range(0, len(stream) - BLOCK, BLOCK)]
    to_text = recogniser()
    texts = [to_text(a) for a, _ in utterances(blocks, MIC_FLOOR)]
    for t in texts:
        print(f"  heard: {t!r} -> {parse(t)}")
    assert [parse(t) for t in texts] == [want for _, want in orders], "an order was misheard"
    assert not list(utterances([np.zeros(BLOCK, np.float32)] * 50, MIC_FLOOR)), "silence produced an order"

    def spoken(text, voice):
        with tempfile.TemporaryDirectory() as tmp:
            subprocess.run(["say", "-v", voice, "-o", f"{tmp}/a.wav", "--data-format=LEI16@16000", text], check=True)
            return read_wav(f"{tmp}/a.wav")

    rng = np.random.default_rng(1)
    room = np.zeros(2600, np.float32)
    room[[0, 90, 230, 900, 2400]] = 0.6, 0.25, -0.1, 0.08, 0.05   # direct sound and echoes out to 150 ms

    def interrupts(robot_says, user=None, at=1.0):
        said = spoken(robot_says, "Samantha")
        mix = np.concatenate([np.zeros(2000, np.float32), np.convolve(said, room)])   # 125 ms late
        mix += rng.normal(0, 0.001, len(mix)).astype(np.float32)
        if user:
            u = spoken(user, "Daniel")[:len(mix) - int(at * RATE)] * 0.5   # quieter than the robot
            mix[int(at * RATE):int(at * RATE) + len(u)] += u
        cut = [mix[i:i + BLOCK] for i in range(0, len(mix) - BLOCK, BLOCK)]
        got = []
        for a, _ in utterances(cut, MIC_FLOOR, speaking=lambda: True):
            a = remove_own_voice(a, said)
            if loudest(a) > MIC_FLOOR:
                got.append(barge_in(to_text(a), robot_says))
        return [g for g in got if g]

    for line in ["Stopped at 4 centimetres. All pads are on.", "Ready on the wall. " + HELP,
                 "I am at 12 centimetres, climbing to 40. 3 pads are on the wall. The weakest is front-right with 13 newtons."]:
        assert not interrupts(line), f"the robot interrupted itself with: {line}"
    soft = rng.normal(0, 0.001, 5 * RATE).astype(np.float32)
    word = spoken("stop", "Daniel") * 0.12
    soft[2 * RATE:2 * RATE + len(word)] += word
    got = [barge_in(to_text(remove_own_voice(a, spoken("Ready on the wall.", "Samantha"))), "Ready on the wall.")
           for a, _ in utterances([soft[i:i + BLOCK] for i in range(0, len(soft) - BLOCK, BLOCK)], MIC_FLOOR, speaking=lambda: True)
           if loudest(a) > MIC_FLOOR]
    assert got == ["stop"], "a quiet stop without echo was not heard"
    long = "I am at 12 centimetres, climbing to 40. 3 pads are on the wall. The weakest is front-right with 13 newtons."
    heard = [bool(interrupts(long, "stop", at)) for at in (0.5, 1.0, 1.3, 1.8, 2.6, 3.4)]
    print(f"  stop spoken over the robot at 6 moments: heard {sum(heard)} of 6")
    assert all(heard), "stop spoken over the robot was not heard"
    print(f"microphone self-test passes: {len(texts)} spoken orders understood")


# -- speaking ----------------------------------------------------------------
class Voice:
    def __init__(self, mute):
        self.lines, self.last = [], {"You": "", "Robot": ""}
        self.q, self.speaking, self.now, self.proc = queue.Queue(), False, "", None
        self.audio, self.wav = np.zeros(0, np.float32), str(Path(tempfile.gettempdir()) / f"electroclimb-say-{id(self)}.wav")
        if not mute and shutil.which("say"):
            threading.Thread(target=self._speak, daemon=True).start()
        else:
            self.q = None

    def _speak(self):
        while True:                                   # one sentence at a time, never overlapping
            self.now = self.q.get()                   # the microphone needs to know what the robot says
            self.speaking = True
            subprocess.run(["say", "-r", "190", "-o", self.wav, "--data-format=LEI16@16000", self.now])
            self.audio = read_wav(self.wav)
            self.proc = subprocess.Popen(["afplay", self.wav])
            self.proc.wait()
            self.speaking = not self.q.empty()

    def hush(self):
        """Cut the robot off mid-sentence (it was interrupted)."""
        if self.q:
            with contextlib.suppress(queue.Empty):
                while True:
                    self.q.get_nowait()
            if self.proc and self.proc.poll() is None:
                self.proc.terminate()

    def note(self, who, text):
        self.lines.append(f"{who}: {text}")
        self.last[who] = text
        print(f"{who}: {text}", flush=True)

    def say(self, text):
        self.note("Robot", text)
        if self.q:
            self.q.put(text)


# -- the camera stand-in and the crack detector ------------------------------
class Camera:
    def __init__(self, seed=3):
        self.seed, self.model, self.pools = seed, None, None
        try:
            import torch
            from torch.utils.data import random_split
            import crack_model
            items = crack_model.Cracks().items
            if len(items) == 40_000:                  # same fixed split as training: use only the held-out part
                test = random_split(range(40_000), [32_000, 4_000, 4_000], generator=torch.Generator().manual_seed(0))[2]
                self.pools = {k: [items[i][0] for i in test.indices if items[i][1] == k] for k in (0, 1)}
                self.model, self.cm = crack_model.load_model(), crack_model
        except (ImportError, FileNotFoundError):
            pass

    def shoot(self, height_cm, out):
        """Photo of the wall at this height -> finding dict, or None without the dataset."""
        if not self.model:
            return None
        import torch
        from PIL import Image
        band = int(round(height_cm / BAND_CM))        # nearest band
        rng = np.random.default_rng(self.seed * 1000 + band)
        truth = int(rng.random() < 0.3)               # 30 % of the bands are cracked
        src = self.pools[truth][rng.integers(len(self.pools[truth]))]
        img = Image.open(src)
        with torch.no_grad():
            p = float(torch.sigmoid(self.model(self.cm.to_tensor(img)[None]).squeeze()))
        name = f"photo_{height_cm:05.1f}cm.jpg"
        img.save(out / name)
        return {"height_cm": round(height_cm, 1), "crack_tiles": int(p > 0.5), "probability": round(p, 3),
                "photo": name, "truth": bool(truth)}


# -- the robot ---------------------------------------------------------------
class Robot:
    def __init__(self):
        self.env = ClimbEnv(tilt_deg=90, randomise=False, fail_prob=0.0, episode_s=1e6)
        self.env.reset()
        self.gait = crawl_gait.CrawlGait(self.env)
        self.dt = self.env.model.opt.timestep * self.env.frame_skip
        self.todo = []                                # waypoints: (height_cm, take a photo there?)
        self.frozen, self.fell, self.t = False, False, 0.0
        self.action = np.concatenate([np.zeros(12), np.ones(4)])

    @property
    def height(self):
        return (self.env.data.xpos[self.env.torso, 0] - self.env.start_x) * 100

    @property
    def at(self):
        return int(round(self.height)) + 0            # whole centimetres for speech, never "-0"

    def pressures(self):
        from balance import pad_loads
        return pad_loads(self.env)[0]

    def step(self):
        """One control step (20 ms). Returns a waypoint when the robot has just arrived at it."""
        arrived = None
        moving = self.todo and not self.frozen
        boundary = (self.gait.t % self.gait.cycle) < self.dt * 1.5
        if moving and boundary:                       # between gait cycles: all four pads down
            left = (self.todo[0][0] - self.height) / 100
            if abs(left) < 0.01:
                arrived, moving = self.todo.pop(0), False
            else:                                     # shorten the last stride so it stops at the target
                crawl_gait.STRIDE = float(np.clip(left, -STRIDE, STRIDE))
        if moving:
            self.action = self.gait(self.env._foot_contacts())
        else:
            self.action[12:] = 1.0                    # standing still: every pad on
        self.env.step(self.action)
        self.t += self.dt
        z = self.env.data.xpos[self.env.torso, 2]
        self.fell = bool(z > 0.35 or z < 0.02)
        return arrived


class Session:
    def __init__(self, mute=False, out=None):
        self.out = out or HERE / "missions" / time.strftime("voice-%Y%m%d-%H%M%S")
        self.out.mkdir(parents=True, exist_ok=True)
        self.voice, self.robot, self.camera = Voice(mute), Robot(), Camera()
        self.findings, self.why, self.llm, self.done = [], "I have not been given a job yet.", None, False

    # what the robot does with a sentence
    def hear(self, text):
        v, r = self.voice, self.robot
        v.note("You", text)
        parsed = parse(text) or self.ask_claude(text)
        if not parsed:
            return
        intent, a = parsed
        if intent in ("quit", "stop"):
            v.hush()
        if intent == "quit":
            v.say("Goodbye.")
            self.done = True
        elif intent == "stop":
            r.frozen, self.why = True, "You told me to stop."
            v.say(f"Stopped at {r.at} centimetres. All pads are on.")
        elif intent == "resume":
            r.frozen = False
            v.say("Continuing." if r.todo else "Nothing left to do. Give me a new order.")
        elif intent == "goto":
            target = r.height + a["cm"] if a["relative"] else a["cm"]
            capped = float(np.clip(target, 0, MAX_CM))
            r.todo, r.frozen = [(capped, False)], False
            limit = f" The wall ends at {MAX_CM:.0f} centimetres, so I go to {capped:.0f}." if capped != target else ""
            way = "Climbing" if capped > r.height else "Coming down"
            v.say(f"{way} to {capped:.0f} centimetres. About {abs(capped - r.height) / 0.9:.0f} seconds.{limit}")
        elif intent == "inspect":
            top, step = min(a["top"], MAX_CM), max(a["step"], 2.0)
            stops = [h for h in np.arange(step, top + 0.01, step) if h > r.height + 1]
            if not stops:
                return v.say("I am already above that. Tell me to come down first.")
            r.todo, r.frozen = [(float(h), True) for h in stops], False
            v.say(f"I will take {len(stops)} photos, every {step:.0f} centimetres up to {stops[-1]:.0f}.")
        elif intent == "photo":
            self.photo()
        elif intent == "status":
            p = r.pressures()
            doing = "stopped" if r.frozen else f"heading to {r.todo[0][0]:.0f} centimetres" if r.todo else "waiting"
            weak = int(np.argmin(np.where(p > 0, p, np.inf))) if (p > 0).any() else 0
            v.say(f"I am at {r.at} centimetres, {doing}. {int((p > 0).sum())} pads are on the wall. "
                  f"The weakest is {LEG[FEET[weak]]} with {p[weak]:.0f} newtons.")
        elif intent == "why":
            v.say(self.why)
        elif intent == "grip":
            p = r.pressures()
            v.say(", ".join(f"{LEG[f]} {x:.0f}" for f, x in zip(FEET, p)) + " newtons. "
                  + ("All four pads hold." if (p > 3).all() else "A pad is weak or in the air; I would not climb yet."))
        elif intent == "report":
            self.report()

    def ask_claude(self, text):
        """Free sentences: questions go to the manual assistant, orders to the mission planner."""
        v = self.voice
        try:
            if self.llm is False:
                raise SystemExit
            if re.match(r"(what|how|is|are|should|does|do|can|which|when)\b", text.lower()) or text.endswith("?"):
                import rag
                answer, sources = rag.answer(text, rag.load().search(text))
                self.llm = True
                v.say(" ".join(re.split(r"(?<=[.!?])\s+", re.sub(r"\s*\[\d+\]", "", answer))[:2]))
                print("   sources:", "; ".join(sources))
                return None
            import mission
            plan = mission.plan_with_claude(text)
            self.llm = True
            self.robot.todo = [(w.height_cm, w.action == "inspect") for w in plan.waypoints]
            self.robot.frozen = False
            v.say(plan.summary)
        except (SystemExit, FileNotFoundError, Exception) as e:    # no key, no index, network: stay usable
            if self.llm is None:
                self.llm = False
            if not isinstance(e, SystemExit):
                print(f"   (Claude call failed: {type(e).__name__})")
            v.say("I did not understand that. " + HELP)
        return None

    def photo(self):
        r, v = self.robot, self.voice
        f = self.camera.shoot(r.height, self.out)
        if f is None:
            return v.say(f"Photo taken at {r.at} centimetres. The crack check needs the dataset, so I did not check it.")
        self.findings = [x for x in self.findings if round(x["height_cm"] / BAND_CM) != round(f["height_cm"] / BAND_CM)] + [f]
        (self.out / "findings.json").write_text(json.dumps(self.findings, indent=1))
        sure = f["probability"] if f["crack_tiles"] else 1 - f["probability"]
        v.say(f"{'Crack found' if f['crack_tiles'] else 'No crack'} at {f['height_cm']:.0f} centimetres. "
              f"I am {sure * 100:.0f} percent sure.")

    def report(self):
        v = self.voice
        cracks = sorted(f["height_cm"] for f in self.findings if f["crack_tiles"])
        if not self.findings:
            v.say("No photos yet, so there is nothing to report.")
        elif cracks:
            v.say(f"I took {len(self.findings)} photos and found cracks at " + ", ".join(f"{h:.0f}" for h in cracks) + " centimetres.")
        else:
            v.say(f"I took {len(self.findings)} photos and found no cracks.")
        md = ["# Voice mission report", "", "| Height | Result | Confidence | Photo |", "|---|---|---|---|"]
        md += [f"| {f['height_cm']:.0f} cm | {'crack' if f['crack_tiles'] else 'clean'} | "
               f"{(f['probability'] if f['crack_tiles'] else 1 - f['probability']) * 100:.0f} % | {f['photo']} |"
               for f in sorted(self.findings, key=lambda f: f["height_cm"])]
        if self.llm and self.findings:                # written defect report with manual citations
            import rag
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                rag.report(self.out / "findings.json")
            md += ["", "## Assessment", "", buf.getvalue()]
        md += ["", "## Transcript", ""] + [f"- {line}" for line in v.lines]
        (self.out / "report.md").write_text("\n".join(md) + "\n")
        print(f"   report: {self.out / 'report.md'}")

    # one 20 ms tick of the world
    def tick(self):
        r, v = self.robot, self.voice
        arrived = r.step()
        if r.fell:
            v.say("I lost the wall and fell.")
            self.done = True
        elif arrived:
            self.why = f"I reached {arrived[0]:.0f} centimetres, which was my target."
            if arrived[1]:
                self.photo()
            if not r.todo:
                v.say(f"Done. I am at {r.at} centimetres with all four pads on.")


# -- running it --------------------------------------------------------------
def run_script(session, script, on_tick=None):
    """Items run one after another when the robot is idle; '+3 stop' fires 3 s after the item before it."""
    items = [s.strip() for s in script.split(";") if s.strip()]
    r, last = session.robot, 0.0
    while not session.done and (items or r.todo):
        if items:
            m = re.match(r"\+(\d+(?:\.\d+)?)\s+(.*)", items[0])
            if (m and r.t - last >= float(m.group(1))) or (not m and (not r.todo or r.frozen)):
                session.hear(m.group(2) if m else items[0])
                items.pop(0)
                last = r.t
        session.tick()
        if on_tick:
            on_tick(session)
    return session


def run_live(session, speed, mic=False, mic_level=None):
    inbox = queue.Queue()
    threading.Thread(target=lambda: [inbox.put(line) for line in sys.stdin], daemon=True).start()
    if mic:
        threading.Thread(target=listen, args=(inbox, session.voice, mic_level), daemon=True).start()
    session.voice.say("Ready on the wall. " + HELP)
    while not session.done:
        start = time.time()
        while not inbox.empty():
            session.hear(inbox.get().strip() or "stop")   # a bare Return is the sure stop
        session.tick()
        time.sleep(max(0.0, session.robot.dt / speed - (time.time() - start)))
    time.sleep(2.5)                                   # let the last sentence finish


def record(path, script):
    """Scripted mission as a captioned GIF for the README."""
    import mujoco
    from PIL import Image, ImageDraw
    from render_media import apple_font, make_camera, show_voltage
    s = Session(mute=True)
    size, font, frames, n = 340, apple_font(12), [], [0]
    renderer = mujoco.Renderer(s.robot.env.model, size, size)

    def grab(sess):
        n[0] += 1
        if n[0] % 25:
            return
        env = sess.robot.env
        show_voltage(env)
        renderer.update_scene(env.data, make_camera((0.15, 0, 0.05), 0.85, -34, 58))
        im = Image.new("RGB", (size, size + 58), (20, 22, 26))
        im.paste(Image.fromarray(renderer.render().copy()).rotate(90), (0, 0))
        d = ImageDraw.Draw(im)
        d.text((10, size + 8), "You:  " + sess.voice.last["You"][:48], fill=(255, 170, 60), font=font)
        d.text((10, size + 30), "Robot:  " + sess.voice.last["Robot"][:42], fill=(255, 255, 255), font=font)
        d.text((size - 70, 8), f"{sess.robot.height:4.1f} cm", fill=(255, 255, 255), font=font)
        frames.append(im)

    run_script(s, script, grab)
    for _ in range(12):
        grab(s) or frames.append(frames[-1])
    pal = [f.convert("P", palette=Image.ADAPTIVE, colors=48) for f in frames]
    pal[0].save(path, save_all=True, append_images=pal[1:], duration=100, loop=0, optimize=True)
    print(f"saved {path}: {len(frames)} frames, {Path(path).stat().st_size / 1e6:.1f} MB")


def selftest():
    assert parse("Climb to 40 cm") == ("goto", {"cm": 40.0, "relative": False})
    assert parse("climb another 20 centimetres") == ("goto", {"cm": 20.0, "relative": True})
    assert parse("climb 2 metres") == ("goto", {"cm": 200.0, "relative": True})
    assert parse("please come down") == ("goto", {"cm": 0.0, "relative": False})
    assert parse("Inspect every 10 cm up to 30 cm.") == ("inspect", {"step": 10.0, "top": 30.0})
    assert parse("STOP!")[0] == "stop" and parse("how high are you?")[0] == "status"
    assert parse("why did you stop?")[0] == "why"
    assert parse("sing me a song") is None
    s = run_script(Session(mute=True), "climb to 6 cm; +1 stop; +1 status; resume; climb 2 metres; +3 come down to 4 cm; report")
    r = s.robot
    assert not r.fell and abs(r.height - 4) < 1.5, f"should end near 4 cm, is at {r.height:.1f}"
    assert any("Stopped" in line for line in s.voice.lines) and any("wall ends" in line for line in s.voice.lines)
    print(f"voice self-test passes: ended at {r.height:.1f} cm after {r.t:.0f} s of simulation")


if __name__ == "__main__":
    args = sys.argv[1:]
    opt = lambda name: args[args.index(name) + 1] if name in args else None
    if "--selftest" in args:
        selftest()
    elif "--mictest" in args:
        mictest()
    elif "--gif" in args:
        record(opt("--gif"), opt("--script"))
    elif "--script" in args:
        run_script(Session(mute="--mute" in args), opt("--script"))
    else:
        run_live(Session(mute="--mute" in args), float(opt("--speed") or 1), "--mic" in args,
                 float(opt("--mic-level")) if opt("--mic-level") else None)
