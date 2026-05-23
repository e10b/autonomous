class State():
    def __init__(self):
        pass

    def on_enter(self):
        pass

    def on_exit(self):
        pass

    def step(self):
        pass

    def log(self, msg: str, logtype: str = "info"):
        state_name = self.__class__.__name__
        if logtype == "info":
            self.shared_data.logger.info(f"[{state_name}] {msg}")
        elif logtype == "error":
            self.shared_data.logger.error(f"[{state_name}] {msg}")

class StateInstance():
    start_state             = None
    new_controller_state    = None
    new_obstacle_avoidance_controller_state = None
    drive_state             = None
    validating_state        = None
    goal_state              = None
    retry_state             = None
    spin_state              = None
    forward_spiral_state    = None
    backward_spiral_state   = None
    go_to_ar_state          = None
    go_to_obj_state         = None
    circle_state            = None
    heartbeat_state         = None