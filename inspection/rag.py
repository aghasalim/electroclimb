#!/usr/bin/env python3
"""Inspection assistant: retrieval-augmented answers with page citations.

Sources: the FHWA Bridge Inspector's Reference Manual (2023, FHWA-NHI-23-024, US government
work, put it at data/docs/fhwa_birm_2023.pdf) and this project's own docs. Retrieval is BM25
over page chunks; Claude answers only from the retrieved passages and cites each one
(search_result blocks), so every claim points to a manual page.

    ../.venv/bin/python rag.py index                                   # build data/rag_index.json
    ../.venv/bin/python rag.py ask "How is crack width measured on concrete?"
    ../.venv/bin/python rag.py report findings.json                    # defect report for a climb

Needs an Anthropic API key (ANTHROPIC_API_KEY or `ant auth login`). Each question costs one
Claude Opus 5.5 request.
"""
import json
import math
import re
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
INDEX = ROOT / "data" / "rag_index.json"
PDFS = {ROOT / "data" / "docs" / "fhwa_birm_2023.pdf": "FHWA Bridge Inspector's Reference Manual (2023)"}
DOCS = [ROOT / "docs" / "HOW_IT_WORKS.md", ROOT / "hardware" / "foot" / "README.md"]
CHUNK = 1500                 # characters per chunk
TOP_K = 8
MODEL = "claude-opus-5-5"
WORD = re.compile(r"[a-z0-9]+")


def tokens(text):
    return WORD.findall(text.lower())


def build_index():
    from pypdf import PdfReader
    chunks = []
    for pdf, title in PDFS.items():
        for n, page in enumerate(PdfReader(pdf).pages, 1):
            text = " ".join((page.extract_text() or "").split())
            for i in range(0, len(text), CHUNK):
                if len(text[i:i + CHUNK]) > 200:
                    chunks.append({"source": f"{title}, page {n}", "title": title, "text": text[i:i + CHUNK]})
    for md in DOCS:
        text = md.read_text()
        for i in range(0, len(text), CHUNK):
            chunks.append({"source": str(md.relative_to(ROOT)), "title": f"ElectroClimb {md.name}",
                           "text": text[i:i + CHUNK]})
    INDEX.write_text(json.dumps(chunks))
    print(f"indexed {len(chunks):,} chunks -> {INDEX}")


class BM25:
    """Okapi BM25 ranking (k1 = 1.5, b = 0.75)."""

    def __init__(self, chunks, k1=1.5, b=0.75):
        self.chunks, self.k1, self.b = chunks, k1, b
        self.tf = [Counter(tokens(c["text"])) for c in chunks]
        self.len = [sum(t.values()) for t in self.tf]
        self.avg = sum(self.len) / len(self.len)
        df = Counter(w for t in self.tf for w in t)
        n = len(chunks)
        self.idf = {w: math.log(1 + (n - d + 0.5) / (d + 0.5)) for w, d in df.items()}

    def search(self, query, k=TOP_K):
        q = set(tokens(query))
        scores = []
        for tf, ln in zip(self.tf, self.len):
            s = 0.0
            for w in q & tf.keys():
                f = tf[w]
                s += self.idf[w] * f * (self.k1 + 1) / (f + self.k1 * (1 - self.b + self.b * ln / self.avg))
            scores.append(s)
        best = sorted(range(len(scores)), key=scores.__getitem__, reverse=True)[:k]
        return [self.chunks[i] for i in best if scores[i] > 0]


def load():
    return BM25(json.loads(INDEX.read_text()))


def answer(question, hits, extra=""):
    """Ask Claude, giving it only the retrieved passages; returns (text with [n] marks, sources)."""
    import anthropic
    client = anthropic.Anthropic()
    try:
        client.models.retrieve(MODEL)
    except anthropic.AuthenticationError:
        raise SystemExit("The Anthropic API key was rejected. Set a valid ANTHROPIC_API_KEY (console.anthropic.com).")
    content = [{"type": "search_result", "source": h["source"], "title": h["title"],
                "content": [{"type": "text", "text": h["text"]}], "citations": {"enabled": True}}
               for h in hits]
    content.append({"type": "text", "text": extra + question})
    response = client.beta.messages.create(
        model=MODEL, max_tokens=16000,
        system=("You help inspect concrete structures for a wall-climbing inspection robot. Answer only from "
                "the search results given; if they don't cover the question, say so. Be concise and practical."),
        messages=[{"role": "user", "content": content}],
        thinking={"type": "adaptive"},
        betas=["server-side-fallback-2026-07-01"], fallbacks="default",
    )
    if response.stop_reason == "refusal":
        return "The request was declined.", []
    sources, out = [], []
    for block in response.content:
        if block.type != "text":
            continue
        out.append(block.text)
        for c in block.citations or []:
            if c.source not in sources:
                sources.append(c.source)
            out.append(f" [{sources.index(c.source) + 1}]")
    return "".join(out).strip(), sources


def ask(question):
    text, sources = answer(question, load().search(question))
    print(text)
    print("\nSources:")
    for i, s in enumerate(sources, 1):
        print(f"  [{i}] {s}")


def report(findings_path):
    """Defect report for one climb. findings.json: [{"height_cm": 42, "crack_tiles": 3, "photo": "..."}, ...]"""
    findings = json.loads(Path(findings_path).read_text())
    hits = load().search("concrete crack width measure record condition rating defect report spalling")
    extra = ("Findings from the robot's crack detector on one wall (height in cm above the start, number of "
             "image tiles classified as cracked):\n" + json.dumps(findings, indent=1) + "\n\n")
    text, sources = answer("Write a short defect report for these findings: what each finding may mean, what "
                           "an inspector should measure or record next, and how urgent it is.", hits, extra)
    print(text + "\n\nSources:\n" + "\n".join(f"  [{i}] {s}" for i, s in enumerate(sources, 1)))


if __name__ == "__main__":
    cmd = sys.argv[1:2]
    if cmd == ["index"]:
        build_index()
    elif cmd == ["ask"]:
        ask(" ".join(sys.argv[2:]))
    elif cmd == ["report"]:
        report(sys.argv[2])
    elif cmd == ["search"]:                      # retrieval only, no API call
        for h in load().search(" ".join(sys.argv[2:])):
            print(f"- {h['source']}: {h['text'][:110]}...")
    else:
        print(__doc__)
