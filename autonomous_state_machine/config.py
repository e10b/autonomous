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

# Simulation Flag: Turns off degradation for sim, turns it on for real
SIMULATION = True

# Confidence Parameters
CONFIDENCE_THRESHOLD = 60
CONFIDENCE_ADDED_AMOUNT = 100
CONFIDENCE_DEGRADED_AMOUNT = 5

if SIMULATION:
    CONFIDENCE_DEGRADED_AMOUNT = 0
