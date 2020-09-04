

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from sensor_msgs.msg import LaserScan
from breezyslam.sensors import RPLidarA1 as LaserModel

from breezyslam.algorithms import RMHC_SLAM
from roboviz import MapVisualizer

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10
MIN_SAMPLES             = 200

class LidarProcessingNode(Node):

    def __init__(self):
        super().__init__('cont')

        slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)
        viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')

        self.subscriber = self.create_subscription(LaserScan, '/scan', self.lidarCallback,10)
        self.publisher = self.create_publisher(Int64, '/servo', 1)
        print("Setup Finished")

    def lidarCallback(self,data):
        # print("Updating Vehicle Motion")
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment
        self.time_increment = data.time_increment
        self.scan_time = data.scan_time
        self.range_max = data.range_max
        self.range_min = data.range_min
        self.intensities = data.intensities
        self.distances = data.ranges

        #Filtering out points that are outside the max range or Nan.
        self.angles = [i for i in range(int(self.angle_min),int(self.angle_max))]
        if len(self.angles) != len(self.distances):
            self.angles = self.angles[:len(self.distances)]

        if len(distances) > MIN_SAMPLES:
            slam.update(distances, scan_angles_degrees=angles)
            self.previous_distances = self.distances.copy()
            self.previous_angles    = self.angles.copy()

        # If not adequate, use previous
        elif previous_distances is not None:
            slam.update(self.previous_distances, scan_angles_degrees=self.previous_angles)

        self.slam.update(laserScanData)

        x, y, theta = self.slam.getpos()

        slam.getmap(mapbytes)

        if not viz.display(x/1000., y/1000., theta, mapbytes):
            exit(0)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = LidarProcessingNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
