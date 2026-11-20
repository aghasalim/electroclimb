# Launch posts (drafts)

Drafts for announcing ElectroClimb. Nothing here has been posted. Images: [`media/post_square.png`](../media/post_square.png) (1200 × 1200), [`media/social_preview.png`](../media/social_preview.png) (1280 × 640), or a GIF from [`media/`](../media/).

LinkedIn notes: best window Tue–Thu 7:30–9:00; put the repository link in the **first comment**, not in the post; one or two hashtags at the end.

---

## Post 1 · LinkedIn · the launch

*Format: controlled comparison. Goal: comments. Image: `post_square.png` or `climb_wall_front.gif`.*

I trained a wall-climbing robot for more than 40 million simulated steps. It got 2 points better.

Then I changed one thing in the foot design. It got 8 points better.

Same robot. Same 40 test walls. The only difference was where the fix lived.

The robot is ElectroClimb: an open-source quadruped that sticks to concrete, glass and wood with electroadhesive feet. Thin pads at 3 kV, a few microamps, no magnets, no suction.

My plan was simple. Write a basic climbing gait, then let reinforcement learning make it great. 15 training runs later (PPO, a curriculum, a teacher that could see the true grip of every wall) the best policy climbed 82% of my hardest walls. The hand-written gait already did 80%.

So I stopped tuning and looked at why it fell. Three walls failed for one reason: a pad lost its voltage for 20 milliseconds. No policy reacts in 20 ms.

The fix was to split each pad in two halves with their own driver. A drop-out now loses half a pad. 88%, with no learning at all.

I'm a third-year student, and this was the most useful lesson of the project so far: I was asking the software to solve a hardware problem.

Everything is still simulation. The real robot comes next.

Where did you last try to fix in software what belonged in hardware?

#robotics #reinforcementlearning

**First comment:** Code, simulation, the detailed model and all results (including the runs that failed): github.com/aghasalim/electroclimb

---

## Post 2 · LinkedIn · WallScan

*Format: curiosity gap. Goal: comments. Image: `wallscan.png`.*

My robot's feet hold it on the wall with electricity. Last week I asked a different question: could the same feet see inside the wall?

Each foot is a comb of copper electrodes. Today the robot reads one number from it: am I touching or not.

But concrete, air, water and steel all change an electric field in different ways. So I split the comb into 8 strips in simulation and measured between every pair. 36 numbers per footstep.

A small neural network learned to read them. On 2,400 footsteps it had never seen, it told sound concrete from a void, a wet zone or a rebar 91% of the time, and placed the defect within about 1 mm in depth. With the single number the robot reads today: 32%.

Capacitive probes for concrete exist. Electroadhesive pads that sense their own contact exist. I could not find anyone using a climbing robot's adhesion pads to scan the wall it is holding on to.

Now the honest part. This is a 2D simulation. Nothing is measured on hardware. Reading picofarads next to 3 kV may turn out to be very hard, and it only sees 1 to 2 cm deep.

It might not survive the first bench test. I'm building that test next.

If you work in non-destructive testing: what would break this first?

#robotics #structuralhealthmonitoring

**First comment:** Write-up with sources, limits and the code: github.com/aghasalim/electroclimb/blob/main/docs/WALLSCAN.md

---

## Post 3 · LinkedIn · thank you, Feetech

*Format: named gratitude. Goal: reposts. Post it when the servos arrive, with a photo of them.*

In November I emailed a servo company in Shenzhen from Kortrijk, Belgium.

I asked for 12 servos for a robot that only existed in simulation.

FEETECH said yes the next day: 14 STS3215 servos and a bus adapter for ElectroClimb, my open-source wall-climbing robot.

That is every motor the first prototype needs. Each leg gets three: one to swing sideways, one at the hip, one at the knee.

I had sent the request expecting silence. I'm one student, not a team with logos on a race car. A clear ask, a public repository and honest numbers went further than I thought.

Thank you, Celina and the FEETECH team. The build log starts when the box lands.

#robotics #opensource

*Check before posting: tag FEETECH's page, and confirm with Celina that she is happy to be named.*

---

## Short post · X / Mastodon / Bluesky

I'm building ElectroClimb: an open-source robot that walks up concrete, glass and wood on electroadhesive feet (3 kV, microamps, no magnets).

So far in simulation: climbs a vertical wall, RL + hand-written gait at 88% on hard walls, crack detection at 99%.

Hardware next. github.com/aghasalim/electroclimb

---

## Repository description and topics

**Description:** Open-source legged robot that climbs concrete, glass and wood on electroadhesive feet. MuJoCo simulation, reinforcement learning, crack detection and a RAG inspection assistant.

**Topics:** `robotics` `legged-robots` `climbing-robot` `electroadhesion` `reinforcement-learning` `mujoco` `sim-to-real` `open-hardware` `infrastructure-inspection` `computer-vision` `rag`
