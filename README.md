# Autonomous Take-Home: Path Planning Track

Welcome! This assessment is designed to measure how you think and problem-solve, not whether you already know ROS or robotics. Novices and experienced candidates are both expected to succeed here — we care much more about your process than your prior background.

**Time expectation:** roughly one day of focused work (~6–8 hours). Stretch goals are optional and unbounded beyond that.

**AI tools are allowed and expected.** See the AI Usage Log section below — how you use AI is part of what we're evaluating, not something to hide.

---

## Part 0: Base Knowledge Primer

### What is ROS 2?

ROS 2 is a framework where independent programs called **nodes** send messages to each other over named channels called **topics**. A **publisher** sends messages; a **subscriber** receives them. For this task, think of a topic as a live data stream you need to visualize, similar conceptually to a websocket feed. Instead of one giant program controlling everything, you have small focused programs — one for motor control, one for GPS, one for camera processing — that communicate over named channels.

### QoS (Quality of Service)

ROS 2 lets you configure delivery guarantees per topic (e.g., `RELIABLE` vs `BEST_EFFORT`).

1. `RELIABLE` guarantees that every message reaches its destination. The publisher keeps re-sending until the receiver acknowledges. If a packet gets lost, the sender resends it.
2. `BEST_EFFORT` sends data without checking it arrives. The publisher transmits once and moves on, spending no extra time or network resources tracking delivery.

A publisher and subscriber on the same topic must have *compatible* settings, or they silently won't connect. The rule: the subscriber may not ask for more than the publisher offers (a `BEST_EFFORT` publisher cannot serve a `RELIABLE` subscriber). Real ROS 2 prints a warning when this happens, and so does our simulator. Read your terminal output.

### Occupancy Grids

An occupancy grid represents the environment as a 2D grid of cells, where each cell is marked free, occupied, or unknown. Path planners search this grid to find a route from a start cell to a goal cell without crossing occupied cells. In the real world, a rover doesn't see the whole map at once — it builds up the grid incrementally as it moves and its sensors observe new areas. This task simulates that incremental view: the grid starts out almost entirely **unknown** and fills in as the rover drives.

A grid is just a flat list of numbers plus a few facts that say how to read it: how wide a cell is (`resolution`), and where in the world the grid sits (`origin`). Turning a world position into a cell, and a cell back into a world position, sounds trivial and is where a surprising number of bugs live. Read the `GridSnapshot` docstring in `sim/messages.py` before you write any code.

### Path Planning — A* (what you'll implement)

A* is a graph search algorithm that finds the shortest path between two points by exploring the most promising paths first, using a cost function that combines "distance traveled so far" with "estimated distance remaining" (a *heuristic*, an optimistic guess at the distance to the goal). Whether you get the *shortest* path depends entirely on that heuristic being optimistic. That is worth understanding, not just copying.

**Theta\*** (a stretch goal, and what we actually use on the rover) is a variant of A* that allows "any-angle" paths instead of only grid-aligned moves, producing smoother, more realistic paths — but it's meaningfully harder to implement, hence stretch-only.

### The rover has a size

A path planner treats the rover as a dimensionless point moving through cell centres. The real rover is a disc. A path that skims the edge of a boulder is fine for a point and impossible for a disc. The standard fix is to *inflate* every obstacle by the rover's radius so that a point moving through the inflated map is safe for the real disc. The outer edge of the arena is a wall in the same sense: the rover's body cannot hang over it or scrape along it, so hugging the outermost row of cells is unsafe. Nothing marks it in the grid for you.

### Replanning

Because the rover's view of the world changes as it moves (new obstacles appear, unknown ground turns out to have boulders on it), a real planner can't just plan once at the start — it has to detect when its current path is no longer valid and replan. This is the core of what makes this task different from a textbook static-grid A* exercise, and it's the core of what makes our real autonomy stack hard. There is a second half to it that is just as important: **not** replanning when the path is still fine. Planning costs compute, and a path that changes every tick makes the rover dither.

### How Yonder actually uses this

Our rover builds a rolling 30m×30m occupancy grid from stereo camera depth data, where each cell has a confidence value that degrades over time/distance from the camera's current view. We run Theta\* with a pure pursuit controller to plan smooth paths around obstacles while continuously remapping as the rover moves and its view updates. This task is a simplified version of that real pipeline.

**Resources:**
- [ROS 2 Publisher/Subscriber tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [A* Pathfinding for Beginners (Red Blob Games)](https://www.redblobgames.com/pathfinding/a-star/introduction.html)
- [ROS 2 QoS docs](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

---

## Starter Repo

### What's in the repo

```
planner_node.py        YOURS — the ROS node: receives grids, decides when to
                       replan, publishes paths, prints monitoring output.

grid_utils.py          YOURS — world <-> cell conversion, obstacle inflation.
search.py              YOURS — the heuristic and A*.
replan.py              YOURS — "where am I on my path?" and "is my path still valid?"
                       These three files are plain Python with no ROS in them,
                       on purpose: they are what the tests exercise.

tests/                 Unit tests for the three files above. Run them as you go.
                       Add your own.

sim/                   PROVIDED — please do not modify anything in here.
  launch.py            Entry point. Run this to start everything.
  simulation.py        The tick loop that ties everything together.
  world.py             The ground truth (which obstacles really exist) and the scenarios.
                       Your planner never sees this.
  grid_feed_publisher.py  The rover's "mapping stack": sensor model -> GridSnapshot.
  rover.py             A minimal rover that drives whatever path you last published.
  scoring.py           Judges your run from the outside. Read it to know how you're scored.
  visualizer.py        Live matplotlib picture of the run.
  messages.py          GridSnapshot and helpers for the nav_msgs/Path you publish.
  rclpy_lite/          A lightweight simulator shim with the same API as real rclpy.
                       Lets you write ROS-style code without installing ROS.
  nav_msgs/  geometry_msgs/  std_msgs/    Standard ROS message types as dataclasses.

requirements.txt       pip dependencies (numpy, matplotlib — only for the visualizer).
AI_LOG.md              Template for your AI usage log (Part 3).
METHODOLOGY.md         Template for how to run your work and your thought process (Part 4).
```

### Familiarisation — read these before you start

**`sim/messages.py`** — the most important file to read first. The `GridSnapshot` docstring defines exactly how the grid is laid out (row-major? which corner is `origin`? which way is `row 0`?) and what every field means. Every bug you can have in the coordinate maths is answered in that docstring.

**`sim/grid_feed_publisher.py`** — how the grid is built each tick, and the QoS it publishes with (this is the thing most likely to silently break your node).

**`sim/rover.py`** — what the rover does with the path you publish. Note what an *empty* path means.

**`sim/scoring.py`** — exactly how your run is judged: what counts as a plan, a replan, and a *needless* replan.

**`sim/rclpy_lite/`** — you don't need to understand this in depth. It's a lightweight shim with the same API as real rclpy (Node, Publisher, Subscriber, QoS). We name it `rclpy_lite` rather than `rclpy` to avoid ambiguity if you have ROS installed. To port your code to a real ROS 2 system, swap `rclpy_lite` → `rclpy` in your imports.

### Running it

You need Python 3.10 or newer (check with `python3 --version`, or `python --version` on Windows) and `git`. The simulator and the tests use only the standard library; numpy and matplotlib are needed only to draw pictures.

1. **Choose how you'll run Python, and install dependencies.** Pick **one** of the two options. Both work; the rest of the steps are the same.

   **Option A: directly in your terminal** (no virtual environment). The simplest route. Type `python3` wherever this README says `python` on macOS and Linux. **On Windows the command is usually `python` (or `py`); if typing `python` opens the Microsoft Store, use `py`.**

   Part 1 needs **nothing installed**. Only the pictures (`--visualize`, `--save-png`) need numpy and matplotlib, so install them if you want pictures:

   - **Ubuntu / WSL:** install the system packages. (A plain `pip install` is refused on recent Ubuntu with `error: externally-managed-environment`.)

     ```bash
     sudo apt install python3-numpy python3-matplotlib python3-tk
     ```

   - **macOS / Windows / other Linux:**

     ```bash
     pip install -r requirements.txt
     ```

   If `pip` says `externally-managed-environment` or you'd rather not touch your system Python, use Option B.

   **Option B: a Python virtual environment.** Keeps everything inside the project and works the same on every OS. On Ubuntu / WSL you may first need `sudo apt install python3-pip python3-venv`.

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate          # Windows PowerShell:  .venv\Scripts\Activate.ps1   (Windows cmd.exe: .venv\Scripts\activate.bat)
   pip install -r requirements.txt
   ```

   On Windows, PowerShell may refuse the activate script with "running scripts is disabled on this system". Either use `cmd.exe` and the `activate.bat` line, or run `Set-ExecutionPolicy -Scope Process Bypass` first (it only affects that one window).

   Your prompt now starts with `(.venv)`. Run the `source` line again in every new terminal, and type `deactivate` to leave. Inside the environment the command is called `python`, which is what the rest of this README uses.

2. **Run the tests** (from the repo root). They all fail at first. Your job is to turn them green:

   ```bash
   python -m unittest -v
   ```

   You should see `FAILED (errors=48)` to begin with. That is correct: 48 tests are waiting for you, and 1 already passes.

3. **Start the simulator** (from the repo root):

   ```bash
   python sim/launch.py --scenario open
   ```

   This loads your `planner_node.py` and runs the whole scenario in a fraction of a second. Until you implement the node, it prints a summary saying what is missing (at first: no subscription to `/grid_feed`). That is expected.

4. **Enable the live visualizer** once your node publishes a path:

   ```bash
   python sim/launch.py --scenario boulders --visualize
   ```

   Opens a matplotlib window showing the rover's map (dark gray is unknown, white is free, black is occupied), your planned path in blue, the trail the rover has driven, and the goal. Replans are marked on the trail. Faint red squares are real obstacles the rover has not seen yet.

   > **No display?** `--visualize` needs one (Windows 11 has one built into WSL through WSLg, and you may also need `sudo apt install python3-tk`). Use `--save-png out.png` instead to write a picture of the final state of a run.

| Command | What it does |
| --- | --- |
| `python sim/launch.py` | Default scenario (`boulders`), headless, prints a summary |
| `python sim/launch.py --scenario NAME` | Pick a world: `open`, `boulders`, `wall`, `popup`, `goal-blocked`, `random` |
| `python sim/launch.py --scenario random --seed 7` | A generated world; every seed is a different one |
| `python sim/launch.py --all` | Every scenario plus random seeds 1–3, as one table |
| `python sim/launch.py --visualize` | Live matplotlib window (`--speed 3` for 3× faster) |
| `python sim/launch.py --save-png out.png` | Run, then save a picture of the final state |
| `python sim/launch.py --noise` | Part 2: noisy sensor (see Part 2A) |
| `python sim/launch.py --no-node` | Simulator only, no planner |

### The feeds

| Topic | Rate | Message | QoS |
| --- | --- | --- | --- |
| `/grid_feed` (you subscribe) | 5 Hz of *simulated* time | `GridSnapshot` (grid + rover position + goal) | check the source |
| `/planned_path` (you publish) | when your plan changes | `nav_msgs/Path` (waypoints in world metres) | your choice |

Some numbers worth knowing (all also in the messages): the arena is 30 m × 30 m, cells are 0.5 m, the rover sees everything within 8 m of itself, drives at 1.5 m/s, and has a planning radius of 0.5 m. The start and goal are the same in every scenario.

**Simulated time.** One tick is 0.2 s of *simulated* time. The simulator does not wait for the wall clock, so a whole run takes a fraction of a second (`--visualize` slows it to real time so you can watch). Any "time" in your monitoring output must be simulated time (`msg.header.stamp`), not `time.time()`.

### What the grid tells you

Each `GridSnapshot` is everything the rover knows at that moment:

- **`occupancy`**: per cell, `-1` unknown, `0` free, `100` occupied.
- **`confidence`**: per cell, 0.0–1.0, how much to trust that occupancy value.

> ### Part 1 uses a perfect sensor. You do not need `confidence` for Part 1.
>
> With the default (clean) feed, every cell inside the rover's sensor range is reported **exactly** as it is in the world. An obstacle is always `occupied`, empty ground is always `free`, and confidence is simply `1.0` for anything the rover has seen and `0.0` for anything it hasn't. Nothing is ever wrong, and nothing decays. **Ignore the `confidence` array entirely in Part 1.**
>
> Confidence-based planning is **Part 2**, which turns on a noisy sensor (`--noise`) where confidence starts to mean something.

What is *not* perfect in Part 1 is the rover's **knowledge**: it only sees within 8 m, so most of the map starts out unknown, and things can change after it last looked.

---

## Part 1: Core Task (required)

Complete `planner_node.py` (and the three pure-Python files it uses) so that it:

1. **Subscribes to `/grid_feed`**, which delivers a new occupancy grid snapshot and the rover's current position at each tick.
2. **Implements A\*** to plan a path from the rover's current position to the goal, respecting occupied cells and the rover's size.
3. **Detects when the current path is no longer valid** — because a cell along the part of the path still ahead of the rover became occupied — and **replans** from the rover's current position when this happens. Don't replan when the path is still fine.
4. **Publishes the current planned path** on `/planned_path` each time it changes.
5. **Provides basic monitoring output** (terminal is fine): current path length, number of replans triggered so far, and time since last replan.

### Suggested order of work

Each step has a check that tells you it's right before you move on, and ends with a **commit** (see "Commit as you go" below).

| Step | What to do | How you know it's done | Commit message |
| --- | --- | --- | --- |
| 1 | Read `sim/messages.py`. Run `python sim/launch.py --scenario open --no-node` and watch the rover sit still. | The `!!` line at the bottom of the summary says why. | *(nothing to commit yet)* |
| 2 | `grid_utils.py`: `world_to_cell`, `cell_to_world`, `inflate`. | `python -m unittest tests.test_grid_utils` | `step 2: grid_utils` |
| 3 | `search.py`: `heuristic`, `astar`. | `python -m unittest tests.test_search` | `step 3: search` |
| 4 | `replan.py`: `next_waypoint_index`, `path_is_valid`. | `python -m unittest tests.test_replan` | `step 4: replan helpers` |
| 5 | `planner_node.py`: subscribe, publish, plan once. Get the rover across the `open` scenario. | `--scenario open` reaches the goal, 1 plan, 0 replans. | `step 5: plan once` |
| 6 | Add the replan logic. Run `boulders`, `wall`. | Reaches the goal, 0 blocked ticks. | `step 6: replanning` |
| 7 | `popup` and `goal-blocked`. | Reaches the goal, 0 blocked ticks, 0 needless replans. | `step 7: popup and goal-blocked` |
| 8 | `--all`, then some `--scenario random --seed N` you haven't tried. | Every row reaches the goal. | `step 8: random seeds` |
| 9 | Write your write-up (in this README), finish `METHODOLOGY.md` and `AI_LOG.md`. | Every question in the write-up has an answer with numbers from your own runs. | `step 9: write-up, methodology and AI log` |

Use `--visualize` (or `--save-png`) constantly. You will learn more from watching one replan happen than from re-reading your code.

**Predict, then run.** Before you first run `boulders`, write down (in your README write-up) how many replans you expect and why, and **commit that prediction on its own** (`prediction: boulders`) before you run it. After the run, compare. If the number surprises you, that surprise is the interesting part.

### Commit as you go

We read your commit history as well as your code. It shows how you worked, and it is the honest record behind your write-up. Commit at the end of each step in the table above, using the message shown.

One-time setup (git refuses to commit until it knows who you are). Use your own name and email:

```bash
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```

At the end of each step:

```bash
git status                  # what changed? nothing surprising?
git add -A
git commit -m "step 2: grid_utils"
```

Rules of the road:

- **Small and honest beats tidy.** A history with `step 6: replanning` followed by `fix: replan fired on the rock behind us` is exactly what we like to see. Fixing your own bug in a later commit is normal.
- **Do not squash, amend or force-push** to make the history look cleaner. Do not commit everything in one go at the end.
- **Commit your AI log as you go too.** When an AI tool gets something wrong, write the `AI_LOG.md` entry in the same commit as the fix.
- Do not edit `sim/` or the provided tests to make things pass. Put your own tests in new files under `tests/`. We grade against our own copy of both.

### What "done" looks like

The summary at the end of a run is your scoreboard:

```
 Result             REACHED THE GOAL in 130 ticks (26.0 s simulated)
 Blocked ticks      0
 Paths published    4   (4 plans, 3 of them replans)
 Needless replans   0
```

- **Result** — did the rover get there.
- **Blocked ticks** — the number of ticks the rover had to emergency-stop because your path led into a real obstacle. This should be **0** everywhere.
- **Replans / needless replans** — a replan is *needless* if the path it replaced was still fine according to the grid you were given (`sim/scoring.py` defines exactly what that means). This should be **0** everywhere. Note that more replans is not the same as needless: a world full of boulders legitimately needs many.

We will also run your planner on worlds you haven't seen (`--scenario random` with other seeds), so don't tune it to the visible scenarios.

### What we're looking for

- Does the planner produce a valid path at each tick, and correctly replan when the grid changes underneath it?
- Does it avoid replanning needlessly on every single tick — only when the current path is actually invalidated?
- Correct pub/sub setup against the provided scaffold.
- Reasonable code structure — is the grid/search logic separated from the ROS plumbing?
- A short README write-up (see "Your write-up") explaining your heuristic choice, your replan-trigger logic, and any design decisions or assumptions.

---

## Part 2: Stretch Goals (optional)

Pick any/all. Partial, well-reasoned attempts are valued over none.

### A. Noisy sensor: confidence-based planning

Run `python sim/launch.py --noise`. Same worlds, same rover, but the sensor is now imperfect, and this is where the `confidence` array starts to matter. Read the `NOISE` table in `sim/grid_feed_publisher.py`; every number is explained. In short:

- Confidence falls with distance: a cell right next to the rover is ≈ 1.0, one at the edge of sensor range ≈ 0.25.
- **Low confidence means the reading is more likely to be wrong**: phantom obstacles that aren't there, and real ones that are missed. A wrong reading usually lasts a single tick.
- A cell that isn't currently being observed loses confidence over time. There are also short sensor **dropouts** where nothing is observed at all.
- `confidence_threshold` (0.6) is the value below which we call a reading *unreliable*.

**Step 1 — observe.** Run your Part 1 planner, unmodified, with `--noise`. What happens to the replan count? Where do the replans come from? (The summary now has a **Phantom replans** line: replans triggered by something in your grid that wasn't really there.) Look at `--visualize`: low-confidence cells fade toward gray.

**Step 2 — fix it.** Use confidence so that you stop chasing phantoms without becoming blind to real obstacles. There is no single right answer; questions worth answering in your write-up:

- Which cells should count as blocking: everything marked occupied? Only cells above the threshold? What about an obstacle you saw clearly a moment ago whose confidence has since decayed. Has it gone away?
- Should a change in confidence, with no change in occupancy, ever trigger a replan? What could a replan achieve if *every* cell has lost confidence (a dropout)?
- Unknown cells also have confidence 0. Are they the same thing as low-confidence cells?

We grade the outcome (reached the goal, 0 blocked ticks, few phantom and needless replans), not one specific technique.

### B. Cost-aware planning

Instead of treating cells as purely free/occupied, use the confidence value as a soft cost (e.g., low-confidence cells are traversable but penalised), so the planner prefers well-observed routes rather than only reacting after a replan is forced. This changes `astar` (cells no longer all cost the same) and, with it, what a good heuristic is. Explain how.

### C. Theta\*

Implement Theta\* instead of (or in addition to) A\* for smoother, any-angle paths. This is a meaningfully harder algorithm — attempt it only if the core task felt comfortable. Note that the rover drives in straight lines between the waypoints you publish, and the simulator checks that against the real world, so a long segment has to be checked for obstacles along its whole length, not just at its ends.

### D. Live Webots simulation

We can provide access to a lightweight version of our Webots simulation environment (via Docker) publishing live occupancy grids instead of this simulator. Flag interest in your README and we'll follow up with setup instructions. This one has real infra dependencies, so we're treating it as fully optional, and there is nothing to do for it in this repo.

---

## Part 3: AI Usage Log (required)

Submit a short `AI_LOG.md` with your code (there is a template in the repo root). For each significant use of AI tools, note:

- What you asked
- What you kept vs. rewrote, and why
- Anything the AI got wrong that you had to catch. A very common one is generating a planner that assumes a static map with no replanning logic at all unless explicitly prompted, since that's the default assumption in most training data and tutorials. There are plenty of others; the scoreboard and the tests are there to help you find them.
- How you verified it actually worked (not just that it ran). Which test, scenario or picture showed you the problem?

This is not graded on whether you used AI — it is graded on whether you can tell us what it got wrong and why you fixed it.

---

## Part 4: METHODOLOGY.md (required)

Edit the `METHODOLOGY.md` in the repo root (there is a template) so it covers:

- **How to run your code.** The exact steps for a reviewer to install everything and run your work from a fresh clone and see it working. Say which Python version and OS you tested on.
- **Your thought process, in bullet points.** Why you made the choices you did, and what your own runs showed.
- **Known limitations.** What doesn't work, and what you'd do next. Being upfront counts in your favor.

The "Your write-up" section of this README answers specific questions with numbers from your runs; `METHODOLOGY.md` is the how-to-run and the reasoning, in your own words. Write it for a teammate who has never seen your code.

## Rubric

| Criterion | What we're scoring |
| --- | --- |
| **Correctness** | Planner finds valid paths, reaches the goal in every scenario (including unseen seeds) with 0 blocked ticks; pub/sub actually connects |
| **Replanning behavior** | Correctly detects invalidated paths and replans; doesn't replan needlessly every tick |
| **Design judgment** | Evidence of intentional choices beyond the minimum ask (heuristic choice, replan-trigger design, code structure) |
| **Handling ambiguity** | How did they resolve underspecified parts of the task? Did they make a reasonable call and explain it? |
| **Understanding, not just output** | Can they explain their heuristic, their replan logic, their tradeoffs? Does the write-up show real comprehension, backed by numbers from their own runs? |
| **AI verification** | Evidence they tested/verified AI-assisted code rather than taking it on faith (from log + code quality + tests they added + commit history) |
| **Stretch engagement** (bonus) | Attempted or completed any stretch goal — even partial attempts count positively |

We don't expect a perfect implementation. Those who show genuine effort and learning are the ones who will have a leg up!

---

## Your write-up

### Prediction and observed behavior

I did not record a boulders prediction before the first run, so I cannot honestly
claim a pre-run number. The observed result was 3 replans. They happened as
previously unknown obstacles entered sensor range and invalidated the remaining
route. In future I would commit the prediction before executing the scenario, as
the exercise requests.

### Planning choices

- **Heuristic:** octile distance: `sqrt(2) * min(dr, dc) + abs(dr - dc)`.
  This is the exact unobstructed cost for an 8-connected grid with unit axial
  moves and `sqrt(2)` diagonal moves, so it is admissible and consistent. An
  overestimating heuristic can pop the goal before a cheaper detour has been
  explored, losing A*'s shortest-path guarantee.
- **Replan trigger:** plan once initially; afterward, locate the waypoint closest
  to the rover and validate only that waypoint and those ahead against the latest
  inflated grid. Replan if one is now occupied or out of bounds. Obstacles behind
  the rover and changes away from the remaining path deliberately do nothing. If
  no route exists, retry only after a cell that was blocked for the failed search
  has cleared.
- **Unknown cells:** treat them as traversable. On the first tick, most of the
  arena—including the distant goal—is unknown. Treating unknown as blocked would
  prevent an initial route and leave the rover unable to explore.
- **Rover size:** inflate observed obstacles by the 0.5 m planning radius, and
  separately mark cell centres too close to the arena boundary. Without this,
  a point-valid path can put the rover body into a boulder or wall.
- **No route:** publish an empty `Path`, which explicitly tells the follower to
  stop. Publishing nothing would leave the previous unsafe path active.
- **Time:** monitoring uses `GridSnapshot.header.stamp`, the simulated clock.
  Wall-clock time is unrelated because a headless run executes faster than real
  time.
- **Ambiguity:** circular inflation was chosen over square inflation because it
  models the disc-shaped rover without blocking diagonal cells beyond the stated
  radius. The edge-wall rule is enforced in the node rather than `inflate`, whose
  tested contract is to grow explicit occupied cells only.

### Results

`python3 sim/launch.py --all` on macOS 15.6.1 with Python 3.14.5:

```text
scenario      result           ticks  blocked  plans  replans  needless
-----------------------------------------------------------------------
open          REACHED            121        0      1        0         0
boulders      REACHED            126        0      4        3         0
wall          REACHED            135        0     14       13         0
popup         REACHED            129        0      5        4         0
goal-blocked  REACHED            164        0      4        2         0
random 1      REACHED            137        0     10        9         0
random 2      REACHED            131        0     10        9         0
random 3      REACHED            139        0      6        5         0
```

Beyond the 49 provided unit tests and the standard `--all` run, I ran random
seeds 4 through 20. All 17 reached the goal with zero blocked ticks and zero
needless replans; completion ranged from 126 to 152 ticks. Unit testing exposed
one design mistake: putting boundary-wall behavior inside `inflate` violated the
helper's contract by changing otherwise unrelated edge cells. Moving that rule
to the planner kept both the pure helper and the rover behavior correct.

No stretch goal was attempted; the submitted implementation focuses on a fully
verified Part 1 planner. In noisy mode, raw occupied readings would still cause
phantom replans, so confidence-aware filtering is the clearest next step.

---

## Submitting

1. **Create a public repository on your own GitHub account.**
2. **Point your clone at it.** Your clone's `origin` is our repo, which you can't push to:

   ```bash
   git remote set-url origin https://github.com/<your-username>/<your-repo>.git
   git push -u origin main
   ```

3. **Check that it's public.** Open your repo's link in a private/incognito browser window. If you can see the code without logging in, so can we.
4. **Send us the link** in the Google Form you'll be asked to fill out.

Your repo should include your code, your write-up (the section above, in this README), your `METHODOLOGY.md`, your `AI_LOG.md`, and your **full commit history** (push all of it; do not squash).
