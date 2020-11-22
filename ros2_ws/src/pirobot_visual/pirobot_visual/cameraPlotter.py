
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
import time
import cv2

from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage, Image
class CameraPlotter(Node):

    def __init__(self):
        super().__init__('motor_controller')

        #Creating the subscribers that read Lidar points
        self.firstCall =False
        self.subscriber = self.create_subscription(CompressedImage, "/camera/image/compressed", self.imageCallback,1)

        #Creating interfaces to plot the Lidar data
        plt.ion()
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(111)

    def imageCallback(self,data):
        # np_arr = np.fromstring(data.data, np.uint8)
        print("Starting new message")
        print(type(data.data))
        np_arr = np.asarray(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, 1)
        cv2.imwrite("test.jpg", image_np)

        self.ax1.clear()
        self.ax1.imshow(image_np)
        self.fig.canvas.draw()


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
        self.subscriber = self.create_subscription(Image, "/camera/image", self.imageCallback,1)

        #Creating interfaces to plot the Lidar data
        plt.ion()
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(111)

        self.bridge = CvBridge()

    def imageCallback(self,msg):
        # np_arr = np.fromstring(data.data, np.uint8)
        print("Starting new message")
        print(msg.data)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        cv2.imwrite("test.jpg", cv_image)

        self.ax1.clear()
        self.ax1.imshow(cv_image)
        self.fig.canvas.draw()

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
