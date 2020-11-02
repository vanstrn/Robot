
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
import argparse

from nav_msgs.msg import OccupancyGrid

class MapPlotter_v1(Node):
    def __init__(self):
        """V1 works based on:
        -Always at 0,0
        -Map is cleared and updated every scan cycle.
        """
        super().__init__('motor_controller')

        #Creating the subscribers that read Lidar points
        self.subscriber = self.create_subscription(OccupancyGrid, "/map", self.mapCallback,10)
        self.get_logger().info("Created Node Publishers and Subscribers")

        #Creating interfaces to plot the Lidar data
        plt.ion()
        self.fig = plt.figure(figsize=(8,8))
        self.ax1 = self.fig.add_subplot(111)

        self.get_logger().info("Created Node Parameters")

    def mapCallback(self,data):
        self.get_logger().info("Updating Figure")
        #Reading the Scan Data
        mapHeight = data.info.height
        mapWidth = data.info.width
        mapDataList = data.data
        print(len(mapDataList))
        mapDataGrid = np.asarray(mapDataList).reshape((mapHeight,mapWidth))


        self.ax1.clear()
        self.ax1.imshow(mapDataGrid)
        self.fig.canvas.draw()


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Arguments for 2 Wheel Driving Node')
    parser.add_argument("--debug",default=False,action="store_true", help="Boolean toggle to print operational debug messages.")
    arg = parser.parse_args()

    minimal_publisher = MapPlotter_v1()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
