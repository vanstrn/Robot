
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from rclpy.qos import qos_profile_sensor_data

class CameraPlotter(Node):

    def __init__(self):
        super().__init__('motor_controller')

        #Creating the subscribers that read Lidar points
        self.firstCall =False
        self.subscriber = self.create_subscription(CompressedImage, "image/compressed", self.imageCallback,qos_profile_sensor_data)
        self.bridge = CvBridge()
        #Creating interfaces to plot the Lidar data
        # plt.ion()
        # self.fig = plt.figure()
        # self.ax1 = self.fig.add_subplot(111)
        print("Finished Setup")

    def imageCallback(self,msg):
        print("Starting new message")
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)

        cv2.imwrite("test.jpg", cv_image)

        # self.ax1.clear()
        # self.ax1.imshow(image_np)
        # self.fig.canvas.draw()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CameraPlotter()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

class CameraPlotter2(Node):

    def __init__(self):
        super().__init__('motor_controller')

        #Creating the subscribers that read Lidar points
        self.firstCall =False
        self.subscriber = self.create_subscription(Image, "image/raw", self.imageCallback,qos_profile_sensor_data)

        #Creating interfaces to plot the Lidar data
        # plt.ion()
        # self.fig = plt.figure()
        # self.ax1 = self.fig.add_subplot(111)
        print("Finished Setup")

        self.bridge = CvBridge()

    def imageCallback(self,msg):
        print("Starting new message")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        cv2.imwrite("test.jpg", cv_image)

        # self.ax1.clear()
        # self.ax1.imshow(cv_image)
        # self.fig.canvas.draw()

def main2(args=None):
    rclpy.init(args=args)

    minimal_publisher = CameraPlotter2()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
