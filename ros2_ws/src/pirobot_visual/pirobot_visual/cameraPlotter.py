
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
import time
import cv2


from sensor_msgs.msg import CompressedImage, Image
class CameraPlotter(Node):

    def __init__(self):
        super().__init__('motor_controller')

        #Creating the subscribers that read Lidar points
        self.firstCall =False
        self.subscriber = self.create_subscription(CompressedImage, "/raspicam_compressed", self.imageCallback,1)

        #Creating interfaces to plot the Lidar data
        plt.ion()
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(111)

    def imageCallback(self,data):
        # np_arr = np.fromstring(data.data, np.uint8)
        print("Starting new message")
        ts = time.time()
        np_arr = np.asarray(data.data)
        print(time.time()-ts)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        print(time.time()-ts)
        self.ax1.clear()
        self.ax1.imshow(image_np)
        self.fig.canvas.draw()
        # self.fig.canvas.draw()
        print(time.time()-ts)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CameraPlotter()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
