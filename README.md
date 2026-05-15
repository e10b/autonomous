# Autonomous State Machine

This ROS 2 package contains the main state machine that orchestrates the autonomous behavior of the rover. It handles global path planning, obstacle tracking, obstacle avoidance, AR tag detection/alignment, and smooth transitions between various operational states.

## Configuration

High-level parameters for the node, including the global map manager and obstacle avoidance, can be configured via `autonomous_state_machine/config.py` (no rebuild required for Python changes, but restart the node).

### Key Configurable Parameters
- **`PERSISTENT_GRID_SIZE`**: Total continuous size of the square map canvas in meters.
- **`OVERLAP_SIZE` / `SHIFT_SIZE`**: Manage the sliding window behavior when the rover approaches the edge of the persistent grid, keeping the rover centered.
- **`GRID_CELL_SIZE`**: The physical resolution of a single grid cell in meters (e.g., 0.10m).
- **`PRINT_ASCII_MAP`**: Set to `True` to print an ASCII representation of the global grid and path to the terminal (useful for debugging).
Use `watch -n 0.5 cat /tmp/rover_map.txt` to see this map.
- **`BACKUP_DURATION`**: Time in seconds the rover back-drives when encountering an immediate obstacle before executing an avoidance maneuver (the cooldown for recalculating paths dynamically scales with this).
- **`SIMULATION`**: Flag to toggle simulation-specific behavior (e.g., automatically turning off obstacle degradation).
- **`CONFIDENCE_*`**: Settings related to probabilistic obstacle tracking (thresholds, increase amounts for hits, decay amounts for free space).

## Known Issues

- **Field of View Degradation Issue**: The local occupancy grid currently provides "free space" grid readings that are actually outside the physical field of view of the sensors. When integrated into the global map, this causes the probability mapping to falsely degrade (clear) tracked obstacle cells that it cannot actually see. 
