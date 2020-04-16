
from picamera import PiCamera
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CameraController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.camera = PiCamera()

    def timer_callback(self):
        return

        self.camera.start_preview()
        time.sleep(5)

        #Taking screenshot
        self.camera.capture('/home/images/image.jpg')

        #Taking Video
        self.camera.start_recording("/home/video/video.h264")

        self.camera.stop_preview()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MotorController()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
