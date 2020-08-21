
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
import argparse

from sensor_msgs.msg import LaserScan

class ContinuousLidarPlotter_v1(Node):

    def __init__(self,offset):
        super().__init__('motor_controller')

        #Creating the subscribers that read Lidar points
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.scanCallback,10)
        self.get_logger().info("Created Node Publishers and Subscribers")
        #Creating interfaces to plot the Lidar data
        plt.ion()
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(111,projection='polar')
        self.ax1.set_ylim(0,4)
        self.offset=offset

        self.get_logger().info("Created Node Parameters")

    def scanCallback(self,data):
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment
        self.range_min = data.range_min
        self.range_max = data.range_max
        self.ranges = data.ranges
        self.time = data.header.stamp.sec

        self.get_logger().info("Updating Figure")
        # print("Updating figure")
        self.ax1.clear()
        angles = np.arange(self.angle_min+self.offset,self.angle_max+self.angle_increment+self.offset,self.angle_increment)
        self.ax1.plot(angles,self.ranges)
        self.ax1.set_ylim(0,4)
        self.fig.canvas.draw()
        # self.fig.canvas.draw()

class DiscreteLidarPlotter_v1(Node):
    def __init__(self,grid_size=[241,241],grid_dim=0.1):
        """V1 works based on:
        -Always at 0,0
        -Map is cleared and updated every scan cycle.
        """
        super().__init__('motor_controller')
        self.grid_size = grid_size
        self.grid_dim = grid_dim

        #Creating the subscribers that read Lidar points
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.scanCallback,10)
        self.get_logger().info("Created Node Publishers and Subscribers")

        #location is static (Allows easy upgrade in future)
        self.loc = [0,0] #
        self.orientation = 0 #radians

        #Creating interfaces to plot the Lidar data
        plt.ion()
        self.fig = plt.figure(figsize=(8,8))
        self.ax1 = self.fig.add_subplot(111)

        self.get_logger().info("Created Node Parameters")

    def scanCallback(self,data):
        self.get_logger().info("Updating Figure")
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
            if dist > 20:
                continue
            x_abs = self.loc[0] + dist*np.cos(angle+self.orientation)
            y_abs = self.loc[1] + dist*np.sin(angle+self.orientation)
            #Coordinates in the grid
            x = round(x_abs/self.grid_dim + self.grid_size[0]/2)
            y = round(y_abs/self.grid_dim + self.grid_size[1]/2)
            #pushing into the grid
            if (x>self.grid_size[0]-1) or (y>self.grid_size[1]-1) or (x<0) or (y<0):
                continue
            self.map[int(x),int(y)] += 1

        #Creating a dot for the Robot.
        self.map[int(self.grid_size[0]/2),int(self.grid_size[1]/2)] = 10

        self.ax1.clear()
        self.ax1.imshow(self.map)
        self.fig.canvas.draw()


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Arguments for 2 Wheel Driving Node')
    parser.add_argument("-o", "--offset",type=float,default=90, help="Right motor topic")
    parser.add_argument("--debug",default=False,action="store_true", help="Boolean toggle to print operational debug messages.")
    arg = parser.parse_args()

    minimal_publisher = ContinuousLidarPlotter_v1(arg.offset)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

def main2(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Arguments for 2 Wheel Driving Node')
    parser.add_argument("--debug",default=False,action="store_true", help="Boolean toggle to print operational debug messages.")
    arg = parser.parse_args()

    minimal_publisher = DiscreteLidarPlotter_v1()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
