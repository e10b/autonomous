# Configuration Parameters for Autonomous State Machine

# Global Map Parameters
PERSISTENT_GRID_SIZE = 60.0
OVERLAP_SIZE = 5.0
SHIFT_SIZE = 30.0
GRID_CELL_SIZE = 0.10

# Debugging
PRINT_ASCII_MAP = True

# Obstacle Avoidance
BACKUP_DURATION = 2.0

# Pointcloud → local grid conversion
# OAK-D optical frame: x=right, y=down, z=forward.
# HEIGHT_MIN/MAX are the y-range (meters below camera) that counts as an obstacle.
# Tune based on camera mount height — reject near-zero (above camera) and large-y (ground plane).
PC_LOCAL_GRID_CELLS = 20   # lateral × forward cells; 20 × 0.10m = 2m range
PC_HEIGHT_MIN = 0.05       # meters below camera — ignore points above this
PC_HEIGHT_MAX = 0.5        # meters below camera — ignore ground plane below this

# Simulation Flag: Turns off degradation for sim, turns it on for real
SIMULATION = False

# Confidence Parameters
CONFIDENCE_THRESHOLD = 60
CONFIDENCE_ADDED_AMOUNT = 100
CONFIDENCE_DEGRADED_AMOUNT = 5

if SIMULATION:
    CONFIDENCE_DEGRADED_AMOUNT = 0
