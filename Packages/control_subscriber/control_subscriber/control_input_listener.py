import rclpy
from rclpy.node import Node
from pymavlink import mavutil

from drone_msgs.msg import Control


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('control_listener')
        self.subscription = self.create_subscription(Control, 'control_input', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        time_execute = 0.1
        self.timer_execute = self.create_timer(time_execute, self.execute_commands)

        self.master = mavutil.mavlink_connection("/dev/ttyACM0", baud=57600)
        # Wait a heartbeat before sending commands
        self.master.wait_heartbeat()
        print("--- Heartbeat heart!")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)
        
        # wait until arming confirmed (can manually check with master.motors_armed())
        print("--- Waiting for the vehicle to arm")
        self.master.motors_armed_wait()
        print('--- Armed!')

        self.pitch = 0
        self.roll = 0
        self.throttle = 0
        self.yaw = 0

    def listener_callback(self, msg):

        self.pitch = msg.pitch
        self.roll = msg.roll
        self.throttle = msg.throttle
        self.yaw = msg.yaw

        self.get_logger().info(f"Control input recieved: \n Roll: {self.roll}\t Pitch: {self.pitch}\t Throttle: {self.throttle}\t Yaw: {self.yaw}")

    def execute_commands(self):
        self.master.mav.manual_control_send(
            self.master.target_system,
            int(self.pitch),         #pitch
            int(self.roll),          #roll
            int(self.throttle),      #throttle
            int(self.yaw),           #yaw
            0)
    def drone_shutdown(self):
        self.master.mav.manual_control_send(
            self.master.target_system,
            0,
            0,
            0,
            0,
            0)
        
        # Disarm
        # master.arducopter_disarm() or:
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)

        # wait until disarming confirmed
        self.master.motors_disarmed_wait()
        print("--- Disarmed!")



def main(args=None):
    rclpy.init(args=args)

    control_listener = MinimalSubscriber()

    try:
        rclpy.spin(control_listener)
    except:
        control_listener.drone_shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()