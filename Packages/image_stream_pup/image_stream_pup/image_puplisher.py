import rclpy
from rclpy.node import Node
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


from std_msgs.msg import String
from sensor_msgs.msg import Image


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('image_pupblisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        timer_period = 0.015  # seconds

        self.bridge = CvBridge()

        cam_port = 0
        self.cam = cv.VideoCapture(cam_port)
        result, image = self.cam.read()

        if result:
            self.get_logger().info("Camera initialized!")
        else:
            self.get_logger().info("Camera not initialized!")
        
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        if self.cam.isOpened():
            _, image = self.cam.read()

            image = cv.resize(image, (700, 500))

            #Lav billede om til sensor_msg Image
            msg = Image()
            msg.header.stamp = Node.get_clock(self).now().to_msg()
            msg.header.frame_id = "Webcam"
            msg.height = np.shape(image)[0]
            msg.width = np.shape(image)[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = False
            msg.step = np.shape(image)[2] * np.shape(image)[1]
            msg.data = np.array(image).tobytes()

            #Publish billede
            self.publisher_.publish(msg)
            self.get_logger().info("Image pupblished")



def main(args=None):
    rclpy.init(args=args)

    image_pupblisher = MinimalPublisher()

    rclpy.spin(image_pupblisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_pupblisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
