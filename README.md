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

## Coordinate Frames and TF Tree

### Expected TF Tree

```
map
 └── odom              (static identity in simulation; EKF output on hardware)
      └── base_link    (robot body; published by sim_localization or the real EKF)
           └── oakd_depth_camera   (static offset; published by rover_localization.launch.py)
```

The `map → odom` transform is a static identity in simulation (published by `rover_localization.launch.py`). On hardware, it is produced by the global EKF (`robot_localization`) via a Navsat transform node. The `odom → base_link` transform is published by `sim_localization.py` in simulation, or by the local EKF on hardware.

### Frame Usage by Component

| Data | Frame | Source / Notes |
|------|-------|----------------|
| `local_odom_position` | **odom** | `/autonomous/localization/odometry/local`; used for all navigation and map work |
| `global_map` origin + updates | **odom** | `GlobalMapManager` is initialized from `local_odom_position` on first pointcloud |
| Theta\* start + goal | **odom** | `get_path()` inputs are `local_odom_position` and `goal_odom` |
| OA waypoints (Theta\* output) | **odom** | Passed to `AlignAndMove`; compared with `local_odom_position` in `align.py` |
| `goal_odom` | **odom** | GPS goal transformed via TF2 `map → odom`; stored in `SharedData.goal_odom` |
| `goal_map_xy` | **map** (GPS/NED) | `GPS_2_NED.gps_to_xy()` output; Python equivalent of the C++ `/fromLL` service |
| `cur_position` | **map** (GPS/NED) | Used only for GPS display; not used in navigation decisions |
| ASCII map robot position | **odom** | `print_ascii_map` called with `local_odom_position` so robot marker is correct |
| Every-tick goal check (`_tick`) | **odom** | `dist(local_odom_position, goal_odom) < 2 m` → `validating_state` |
| OA completion goal check | **odom** | Same distance check in `NewObstacleAvoidanceControllerState` |
| PP steering (`local_odom_position`) | **odom** | `new_waypoints_odom` (TF2-converted) passed to PP; `drive_state.py` uses `local_odom_position` as `cur_pose` |
| `new_waypoints_odom` | **odom** | TF2 map→odom conversion of `new_waypoints[1:]`; set by `_convert_waypoints_to_odom()` in `_tick` |

### Goal Transform Pipeline

```
GPS lat/lon
   │  GPS_2_NED.gps_to_xy()          (on_receive_new_gps_waypoint in utils.py)
   ▼
goal_map_xy  (map / GPS-NED frame)
   │  TF2 lookup_transform('odom','map')  (_transform_goal_to_odom in autonomous_state_machine.py)
   │  triggered once per waypoint update via goal_dirty flag
   ▼
goal_odom  (odom frame)
   │  used by: _tick() goal check, _on_pointcloud() Theta*, OA completion check,
   │           _rebuild_main_controller_from_current_path()
```

In simulation `map ≡ odom` (identity static TF), so `goal_odom == goal_map_xy`. On hardware the EKF may introduce a small drift offset between the two frames over time.

### Subscribing to Odom

The node subscribes to `LOCAL_ODOM_TOPIC = "/autonomous/localization/odometry/local"` (defined in `shared_data.py`). This topic must be published before the rover can navigate — confirm with:

```bash
ros2 topic echo /autonomous/localization/odometry/local
```

---

## Known Issues

- **Field of View Degradation Issue**: The local occupancy grid currently provides "free space" grid readings that are actually outside the physical field of view of the sensors. When integrated into the global map, this causes the probability mapping to falsely degrade (clear) tracked obstacle cells that it cannot actually see. 
