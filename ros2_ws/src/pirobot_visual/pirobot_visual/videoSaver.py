
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from rclpy.qos import qos_profile_sensor_data

class VideoSaver(Node):

    def __init__(self):
        super().__init__('motor_controller')

        #Creating the subscribers that read Lidar points
        self.firstCall =False
        self.subscriber = self.create_subscription(CompressedImage, "image/compressed", self.imageCallback,qos_profile_sensor_data)
        self.bridge = CvBridge()

        size = (640,480)
        pathOut = "video.avi"
        self.writer = cv2.VideoWriter(pathOut,cv2.VideoWriter_fourcc(*'DIVX'), 30, size)

        print("Finished Setup")

    def imageCallback(self,msg):
        print("Starting new message")
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)

        self.writer.write(cv_image)

    def End(self):
        print("Finishing video.")

        self.writer.release()

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = VideoSaver()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        minimal_publisher.End()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
