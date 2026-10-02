# Configuration Parameters for Autonomous State Machine

# Global Map Parameters
PERSISTENT_GRID_SIZE = 30 # 30m minimum for sim rollover testing
OVERLAP_SIZE = 5.0
SHIFT_SIZE = PERSISTENT_GRID_SIZE // 2
GRID_CELL_SIZE = 0.10

# Debugging
WRITE_ASCII_MAP = True                  # write an ASCII view of the global map once per second
ASCII_MAP_PATH = "/tmp/rover_map.txt"   # view with: watch -n 0.5 cat /tmp/rover_map.txt
PUBLISH_COSTMAP = False  # publish global map as nav_msgs/OccupancyGrid for Foxglove

# Simulation Flag: Turns off degradation for sim, turns it on for real
SIMULATION = False

# Navigation controller used by NewControllerState: "pure_pursuit" or "align"
CONTROLLER_TYPE = "pure_pursuit"

# Odometry watchdog: stop the rover and pause the state machine if no local odom for this long (seconds)
ODOM_TIMEOUT = 0.5

# GPS recalibration: re-transform map-frame waypoints to odom after this much travel (meters)
RECALIBRATION_DISTANCE = 10.0

# Obstacle Avoidance
BACKUP_DURATION = 2.0
OA_COOLDOWN_EXTRA = 1.0        # extra seconds added to BACKUP_DURATION for OA re-trigger cooldown
OA_CORRIDOR_HALF_CELLS = 8     # half-width (in grid cells) of the rover body corridor checked for obstacles
OA_ROBOT_RADIUS_M = 1.5        # rover body radius (meters) used to inflate obstacles in pathfinding
MIN_OBSTACLE_SIZE = 4          # minimum connected obstacle size in occupied grid cells before triggering OA

# Pointcloud → local grid conversion
# OAK-D optical frame: x=right, y=down, z=forward.
# HEIGHT_MIN/MAX are the y-range (meters below camera) that counts as an obstacle.
# Tune based on camera mount height — reject near-zero (above camera) and large-y (ground plane).
PC_LOCAL_GRID_CELLS = 20   # lateral × forward cells; 20 × 0.10m = 2m range
PC_HEIGHT_MIN = -1.5       # meters below camera — ignore points above this
PC_HEIGHT_MAX = 1.5       # meters below camera — ignore ground plane below this
PC_MIN_POINTS_PER_CELL = 700 # minimum point count in a cell to mark it as occupied
if SIMULATION:
    PC_MIN_POINTS_PER_CELL = 1  # in sim, we trust the pointcloud more — any points in a cell mark it occupied

# Confidence Parameters
CONFIDENCE_THRESHOLD = 60
CONFIDENCE_ADDED_AMOUNT = 100
CONFIDENCE_DEGRADED_AMOUNT = 0

if SIMULATION:
    CONFIDENCE_DEGRADED_AMOUNT = 0

# Heartbeat / comms-loss recovery
HEARTBEAT_ENABLED = False          # set False when running without basestation
HEARTBEAT_TIMEOUT = 4.0           # seconds of silence before declaring comms lost
