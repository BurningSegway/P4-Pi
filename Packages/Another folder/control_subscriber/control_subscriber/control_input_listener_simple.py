import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import time

from drone_msgs.msg import Control


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('control_listener_simple')
        self.subscription = self.create_subscription(Control, 'control_input', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        time_execute = 0.1
        self.timer_execute = self.create_timer(time_execute, self.execute_commands)

        print("--- Heartbeat heart!")
        time.sleep(0.5)
        
        # wait until arming confirmed (can manually check with master.motors_armed())
        print("--- Waiting for the vehicle to arm")
        time.sleep(0.5)
        print('--- Armed!')
        time.sleep(0.5)

        self.time = self.get_clock().now()
        self.recv_time = self.time.nanoseconds

    def listener_callback(self, msg):

        self.recv_time = msg.stamp
        self.pitch = msg.pitch
        self.roll = msg.roll
        self.throttle = msg.throttle
        self.yaw = msg.yaw

        self.get_logger().info(f"Control input recieved at time: {self.recv_time}\n Roll: {self.roll}\t Pitch: {self.pitch}\t Throttle: {self.throttle}\t Yaw: {self.yaw}")

    def execute_commands(self):
        self.time = self.get_clock().now()
        print(f"Recv Time: {self.recv_time}")
        print(f"Act Time: {self.time.nanoseconds}")
        time_diff = self.time.nanoseconds - self.recv_time
        print(time_diff)
        if time_diff < 400000000:
            print("Motors spinnin")

        else:
            print("Motors stopped")
            raise SystemExit

    def drone_shutdown(self):
        print("--- Disarmed!")



def main(args=None):
    rclpy.init(args=args)

    control_listener_simple = MinimalSubscriber()

    try:
        rclpy.spin(control_listener_simple)
    except KeyboardInterrupt:
        control_listener_simple.drone_shutdown()
    except SystemExit:
        control_listener_simple.drone_shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #control_listener_simple.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()