
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

from sensor_msgs.msg import LaserScan

class DiscreteLidar2D_v1(Node):
    def __init__(self,grid_size=[121,121],grid_dim=0.1):
        """V1 works based on:
        -Always at 0,0
        -Map is cleared and updated every scan cycle.
        """
        super().__init__('motor_controller')
        self.grid_size = grid_size
        self.grid_dim = grid_dim

        #Creating the subscribers that read Lidar points
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.scanCallback,10)

        #location is static (Allows easy upgrade in future)
        self.loc = [0,0] #
        self.orientation = 0 #radians

        #Creating interfaces to plot the Lidar data
        plt.ion()
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(111,projection='polar')

        #Creating a timer which will update the lidar plot.
        self.wait=False
        self.timer =self.create_timer(1,self.plotLidar)

    def scanCallback(self,data):
        print("Begin Scan Localization")
        #Reading the Scan Data
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_increment = data.angle_increment
        range_min = data.range_min
        range_max = data.range_max
        ranges = data.ranges
        time = data.header.stamp.sec

        self.map = np.zeros(self.grid_size)
        angles = np.arange(angle_min,angle_max+angle_increment,angle_increment)
        for dist,angle in zip(ranges,angles):
            #Absolute Coordinates
            x_abs = self.loc[0] + np.cos(angle+self.orientation)
            y_abs = self.loc[1] + np.sin(angle+self.orientation)
            #Coordinates in the grid
            x = round(x_abs/self.grid_dim - self.grid_size[0])
            y = round(y_abs/self.grid_dim - self.grid_size[1])

            #pushing into the grid
            self.map[x,y] += 1
        self.wait=True


    def plotLidar(self):
        print("Updating figure")
        if self.wait:
            self.ax1.clear()
            self.ax1.imshow(self.map)
            self.fig.canvas.draw()
            self.fig.canvas.draw()
        return


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = DiscreteLidar2D_v1()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
