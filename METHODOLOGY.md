# Methodology

## 1. How to run it

Tested on macOS 15.6.1 with Python 3.14.5. The core planner and simulator use
only the standard library; NumPy and matplotlib are needed only for rendering.

From a fresh clone:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m unittest -v
python sim/launch.py --all
```

For one detailed run or an image:

```bash
python sim/launch.py --scenario boulders
python sim/launch.py --scenario boulders --save-png boulders.png
```

The expected test result is 49 passing tests. The `--all` table should show
`REACHED`, zero blocked ticks, and zero needless replans for every row.

## 2. Thought process

- Kept coordinate conversion, grid inflation, search, and replan decisions free
  of ROS so each can be tested independently.
- Used floor for world-to-cell conversion because truncation maps points just
  below a negative-origin boundary into the wrong cell.
- Used circular obstacle inflation at the robot radius, preserving a copy of the
  input grid. Arena boundaries are marked separately as virtual walls.
- Used heap-based A* with octile distance, eight-connected movement, and explicit
  diagonal corner checks. The start may be blocked by inflation so search permits
  leaving it, but never entering a blocked goal.
- Treated unknown space as traversable; otherwise the first partial sensor view
  cannot connect to the distant goal.
- Validated only the untraversed portion of the published path. A failed search
  is retried only if a previously blocked cell clears, avoiding repeated empty
  publications on irrelevant map updates.
- Verified all fixed scenarios, the supplied random seeds 1–3, and additional
  seeds 4–20. Every run reached with zero collisions and needless replans.

## 3. Known limitations

- Noisy-sensor confidence is not yet used. Low-confidence phantom obstacles can
  trigger unnecessary replans under `--noise`; I would add temporal hysteresis
  or require confidence above the message threshold before blocking a cell.
- Paths contain one waypoint per grid cell and are safe but not smooth. Theta*
  or collision-checked line-of-sight simplification would reduce waypoint count.
- Replanning is event-driven by path invalidation, not proactive cost
  improvement; a newly discovered shortcut will not replace a still-safe route.
