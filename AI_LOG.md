# AI Usage Log

## 1. Implementing and verifying the core planner

**What I asked:** Complete the path-planning take-home from the starter
repository, including the required implementation, tests, scenarios, and
write-up.

**What I kept vs. rewrote, and why:** Kept the coordinate conversions, circular
inflation, heap-based octile A*, forward-path validity check, compatible ROS QoS,
and simulated-time monitoring after checking each against the repository's
contracts. Refined the no-route retry logic to react only when a formerly blocked
cell clears, because unrelated map discovery cannot open a route when unknown
cells are already traversable.

**What the AI got wrong that I had to catch:** The first implementation put
arena-boundary blocking inside the general `inflate()` helper. Three unit tests
failed because that helper is expected to change only cells near explicit
obstacles. I moved boundary handling into the planner, where the arena-wall rule
belongs, while leaving `inflate()` faithful to its tested contract.

**How I verified it ran correctly (not just that it compiled):**
`python3 -m unittest -v` initially reported 3 failures, including
`(0, 0) is too far to be inflated`; after separating edge handling it reported
`Ran 49 tests ... OK`. `python3 sim/launch.py --all` reached the goal in all 8
runs with zero blocked ticks and zero needless replans. I also ran random seeds
4–20: all 17 reached, again with zero blocked ticks and zero needless replans.

## 2. Producing the submission documentation

**What I asked:** Fill in the README write-up, methodology, and AI log using
measured results rather than generic claims.

**What I kept vs. rewrote, and why:** Kept the run commands, explanation of
octile admissibility, exact replan policy, test table, limitations, and measured
random-seed range. I explicitly recorded that no prediction was committed before
the first boulders run rather than fabricating one after seeing the result.

**What the AI got wrong that I had to catch:** No additional issue was found in
this documentation pass; values were copied from fresh command output.

**How I verified it ran correctly (not just that it compiled):** Re-ran the full
unit suite and scenario matrix after the final logic change, and compared the
README table to the simulator output.
