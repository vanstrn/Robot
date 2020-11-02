

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import time

from breezyslam.sensors import RPLidarA1 as LaserModel
from breezyslam.algorithms import RMHC_SLAM
from roboviz import MapVisualizer
import numpy as np
import matplotlib.pyplot as plt

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 20
MIN_SAMPLES             = 200

class LidarProcessingNode(Node):

    def __init__(self,visualize):
        super().__init__('cont')
        self.visualize = visualize

        self.slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)
        if self.visualize:
            self.viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')

        self.subscriber = self.create_subscription(LaserScan, '/scan', self.lidarCallback,10)
        self.mapPublisher = self.create_publisher(OccupancyGrid, '/map', 1)
        self.locPublisher = self.create_publisher(Pose, '/pose', 1)
        print("Setup Finished")
        self.previous_distances = None
        self.previous_angles    = None

        self.mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        self.i = True

    def lidarCallback(self,data):
        # print("Received Lidar Message")
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment
        self.time_increment = data.time_increment
        self.scan_time = data.scan_time
        self.range_max = data.range_max
        self.range_min = data.range_min
        self.intensities = data.intensities
        self.distances = list(data.ranges)

        self.angle_max = self.angle_max - self.angle_min
        self.angle_min = 0.0

        #Filtering out points that are outside the max range or Nan.
        self.angles = [i*180.0/np.pi for i in np.arange(self.angle_min,self.angle_max,self.angle_increment)] + [self.angle_max*180.0/np.pi]
        self._distances = [i*1000.0 for i in self.distances]

        self._distances,self.angles = (list(t) for t in zip(*[x for x in zip(self._distances,self.angles) if x[0] <= 5000]))

        # print(max(filteredDistances))
        # print(len(filteredDistances))
        # print(len(filteredAngles))
        # if len(self.angles) != len(self.distances):
        #     self.angles = self.angles[:len(self.distances)]

        # print(len(self.distances)
        # print(self.angles)

        if len(self._distances) > MIN_SAMPLES:
            self.slam.update(self._distances, scan_angles_degrees=self.angles)
            self.previous_distances = self._distances.copy()
            self.previous_angles    = self.angles.copy()

        # If not adequate, use previous
        elif previous_distances is not None:
            self.slam.update(self.previous_distances, scan_angles_degrees=self.previous_angles)

        # self.slam.update(laserScanData)

        x, y, theta = self.slam.getpos()

        self.slam.getmap(self.mapbytes)

        if self.visualize:
            if not self.viz.display(x/1000., y/1000., theta, self.mapbytes):
                exit(0)

        #Publishing Position data
        pose =Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.z = theta
        self.locPublisher.publish(pose)

        #Publishing Map data
        map = OccupancyGrid()
        map.header.stamp.sec = 0
        map.header.stamp.nanosec = 0
        map.header.frame_id = "1"
        map.info.map_load_time.sec = 0
        map.info.map_load_time.nanosec = 0
        map.info.resolution = MAP_SIZE_PIXELS/MAP_SIZE_METERS
        map.info.width = MAP_SIZE_PIXELS
        map.info.height = MAP_SIZE_PIXELS
        map.info.origin.position.x = 5000.0
        map.info.origin.position.y = 5000.0
        mapimg = np.reshape(np.frombuffer(self.mapbytes, dtype=np.uint8), (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))
        norm_const = 100/mapimg.max()
        norm_map = (np.absolute(mapimg.astype(float) * norm_const-100.0)).astype(int)
        # plt.matshow(norm_map)
        # plt.show()
        map.data = norm_map.flatten().tolist()
        self.mapPublisher.publish(map)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = LidarProcessingNode(False)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
