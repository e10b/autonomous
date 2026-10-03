from autonomous_state_machine.states.state_class import State
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class HeartbeatState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        self.log("ENTER — heartbeat lost, returning to last safe position", logtype="warn")
        stop = Twist()
        self.shared_data.auton_publisher.publish(stop)
        led = Float64MultiArray()
        led.data = [255, 0, 0]
        self.shared_data.led_publisher.publish(led)
        self.log("[HEARTBEAT] Published stop command and set LED to red")
        with self.shared_data.mutex:
            if len(self.shared_data.new_waypoints_odom) < 2:
                self.log("no safe position recorded — staying put", logtype="warn")

    def step(self) -> State:
        with self.shared_data.mutex:
            if len(self.shared_data.new_waypoints_odom) < 2:
                return self.state_instance.heartbeat_state
        return self.state_instance.new_controller_state