from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from autonomous_state_machine.states.state_class import State

class StartState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        self.log("ENTER — stopped, LED red, awaiting first waypoint")
        led_msg = Float64MultiArray()
        led_msg.data = [255, 0, 0]
        self.shared_data.led_publisher.publish(led_msg)

        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.shared_data.auton_publisher.publish(stop)

    def step(self):
        return self.state_instance.start_state
