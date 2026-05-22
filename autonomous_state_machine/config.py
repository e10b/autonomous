# Configuration Parameters for Autonomous State Machine

# Global Map Parameters
PERSISTENT_GRID_SIZE = 60 # 60 is good
OVERLAP_SIZE = 5.0
SHIFT_SIZE = PERSISTENT_GRID_SIZE // 2
GRID_CELL_SIZE = 0.10

# Debugging
PRINT_ASCII_MAP = True

# Simulation Flag: Turns off degradation for sim, turns it on for real
SIMULATION = True

# Obstacle Avoidance
BACKUP_DURATION = 2.0
OA_COOLDOWN_EXTRA = 1.0        # extra seconds added to BACKUP_DURATION for OA re-trigger cooldown
OA_CORRIDOR_HALF_CELLS = 8     # half-width (in grid cells) of the rover body corridor checked for obstacles
OA_ROBOT_RADIUS_M = 1.0        # rover body radius (meters) used to inflate obstacles in pathfinding

# Pointcloud → local grid conversion
# OAK-D optical frame: x=right, y=down, z=forward.
# HEIGHT_MIN/MAX are the y-range (meters below camera) that counts as an obstacle.
# Tune based on camera mount height — reject near-zero (above camera) and large-y (ground plane).
PC_LOCAL_GRID_CELLS = 20   # lateral × forward cells; 20 × 0.10m = 2m range
PC_HEIGHT_MIN = -0.5       # meters below camera — ignore points above this
PC_HEIGHT_MAX = 0.25       # meters below camera — ignore ground plane below this
PC_MIN_POINTS_PER_CELL = 1000 # minimum point count in a cell to mark it as occupied
if SIMULATION:
    PC_MIN_POINTS_PER_CELL = 1  # in sim, we trust the pointcloud more — any points in a cell mark it occupied

# Confidence Parameters
CONFIDENCE_THRESHOLD = 60
CONFIDENCE_ADDED_AMOUNT = 100
CONFIDENCE_DEGRADED_AMOUNT = 5

if SIMULATION:
    CONFIDENCE_DEGRADED_AMOUNT = 0
