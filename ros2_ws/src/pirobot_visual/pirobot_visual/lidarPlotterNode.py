
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np

from sensor_msgs.msg import LaserScan

class LidarPlotter(Node):

    def __init__(self):
        super().__init__('motor_controller')

        #Creating the subscribers that read Lidar points
        self.firstCall =False
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.scanCallback,10)

        #Creating interfaces to plot the Lidar data
        plt.ion()
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(111,projection='polar')

        #Creating a timer which will update the lidar plot.
        self.timer =self.create_timer(1,self.plotRanges)


    def scanCallback(self,data):
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment
        self.range_min = data.range_min
        self.range_max = data.range_max
        self.ranges = data.ranges
        self.time = data.header.stamp.sec
        self.firstCall = True
        
    def plotRanges(self):
        if self.firstCall:
            print("Updating figure")
            self.ax1.clear()
            angles = np.arange(self.angle_min,self.angle_max+self.angle_increment,self.angle_increment)
            self.ax1.plot(angles,self.ranges)
            self.fig.canvas.draw()
            self.fig.canvas.draw()
        return


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = LidarPlotter()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
