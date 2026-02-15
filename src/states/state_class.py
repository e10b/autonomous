class State():
    def __init__(self):
        pass

    def on_enter(self):
        pass

    def on_exit(self):
        pass

    def step(self):
        pass

class StateInstance():
    start_state             = None
    new_controller_state    = None
    new_obstacle_avoidance_controller_state = None
    drive_state             = None
    obstacle_recovery_state = None
    validating_state        = None
    goal_state              = None
    retry_state             = None
    spin_state              = None
    forward_spiral_state    = None
    backward_spiral_state   = None
    go_to_ar_state          = None
    go_to_obj_state         = None
    charge_forward_state    = None
    circle_state            = None
    heartbeat_state         = None