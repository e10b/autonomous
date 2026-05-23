from std_msgs.msg import Float64MultiArray
from autonomous_state_machine.states.state_class import State

class StartState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        # Print entering state for debugging
        self.shared_data.logger.info("[StartState] Entering startState...")
        led_msg = Float64MultiArray()
        led_msg.data = [255, 0, 0]
        self.shared_data.led_publisher.publish(led_msg)

        from geometry_msgs.msg import Twist
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.shared_data.auton_publisher.publish(stop)

        # Clear debug file for fresh debug output
        self.shared_data.logger.info("[StartState] Clearing debug.txt file...")
        with open('debug.txt', 'w') as f:
            f.write('')

    def step(self):
        return self.state_instance.start_state
