from state_class import State
from std_msgs.msg import Float64MultiArray
from time import time

class AlignState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        print("Entering AlignState...")

        self.shared_data.mutex.acquire()
        self.shared_data.start_time = time()
        self.shared_data.start_heading = self.shared_data.cur_heading
        self.shared_data.mutex.release()

    def step(self) -> State:
        self.shared_data.mutex.acquire()

        vel_msg = Float64MultiArray()

        # Back up for 2 seconds to get a better view of the gate
        if time() - self.shared_data.start_time < self.shared_data.BACKUP_TIMEOUT:
            vel_msg.data = [-self.shared_data.AUTON_SPEED, -self.shared_data.AUTON_SPEED]
            self.shared_data.auton_publisher.publish(vel_msg)
            self.shared_data.mutex.release()
            return self.state_instance.align_state

        # Spin util both tags are in frame
        if self.shared_data.ar_size <= 0 or\
           self.shared_data.ar_size2 <= 0:
            """if time() - self.shared_data.start_time < 1 + self.shared_data.BACKUP_TIMEOUT and\
               abs(self.shared_data.cur_heading - self.shared_data.start_heading) < self.shared_data.SPIN_TOLERANCE:
                vel_msg.data = [-self.shared_data.SPIN_SPEED, self.shared_data.SPIN_SPEED]"""
            if time() - self.shared_data.start_time < self.shared_data.SPIN_TIMEOUT + self.shared_data.BACKUP_TIMEOUT:
                vel_msg.data = [-self.shared_data.SPIN_SPEED,
                                self.shared_data.SPIN_SPEED]
                self.shared_data.auton_publisher.publish(vel_msg)
                if self.shared_data.ar_size > 0:
                    print("Saw AR Tag Id: 0.0")
                if self.shared_data.ar_size2 > 0:
                    print("Saw AR Tag ID: 1.0")
                self.shared_data.mutex.release()
                return self.state_instance.align_state
            else:
                self.shared_data.mutex.release()
                return self.state_instance.circle_state

        # Spin until both tags are centered
        if (self.shared_data.ar_x_pose + self.shared_data.ar_x_pose2)/2 >= 0.6 or\
           (self.shared_data.ar_x_pose + self.shared_data.ar_x_pose2)/2 <= 0.4:
            if time() - self.shared_data.start_time < self.shared_data.SPIN_TIMEOUT + self.shared_data.BACKUP_TIMEOUT:
                vel_msg.data = [-self.shared_data.SPIN_SPEED,
                                self.shared_data.SPIN_SPEED]
                self.shared_data.auton_publisher.publish(vel_msg)
                self.shared_data.mutex.release()
                return self.state_instance.align_state

        # If the distance between 2 tags is too small or only see 1 tag, go back to that tag
        distbtwntags = abs(self.shared_data.ar_x_pose - self.shared_data.ar_x_pose2)
        if distbtwntags < self.shared_data.AR_TAGS_DIST_TOLERANCE or\
           self.shared_data.ar_size <= 0.01 or\
           self.shared_data.ar_size2 <= 0.01:
            self.shared_data.do_circle_task = True
            print("Should do circle")
            self.shared_data.mutex.release()
            return self.state_instance.spin_state

        self.shared_data.mutex.release()
        return self.state_instance.charge_forward_state