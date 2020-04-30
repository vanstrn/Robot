
# Goal:
#   -Create a ROS2 Node that is able to send verbose GPS information.
#
# Based on:
#   https://gpsd.gitlab.io/gpsd/gpsd_json.html


import argparse
import rclpy
from rclpy.node import Node
import time
from math import modf
import gps

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header



class GPSNode(Node):

    def __init__(self,debug=False):
        super().__init__('gps')
        """Establishing connection to the control device.  """
        self.session = gps.gps("localhost", "2947")
        self.session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

        self.gpsPublisher = self.create_publisher(NavSatFix, '/gps/fix',1)

    def run(self):
        while True:
            #Reading acceleration data
            report = session.next()

            if self.debug: print(report)
            if report["class"] == "TPV":
                if "time" in report:
                    gpsMSG = NavSatFix()
                    gpsMSG.latitude = report["lat"]
                    gpsMSG.longitude = report["lon"]
                    gpsMSG.altitude = report["alt"]
                    gpsMSG.status = report["status"]
                    current_time = modf(time.time())
                    gpsMSG.header.sec = int(current_time[1])
                    gpsMSG.header.nanosec = int(current_time[0] * 1000000000) & 0xffffffff
                    self.gpsPublisher.publish(gpsMSG)
            else:
                if self.debug: print("GPS has not locked onto sattelites")





def main():
    #Createion of
    parser = argparse.ArgumentParser(description='Arguments for Imu Node')
    parser.add_argument("--debug",default=False,action="store_true", help="Boolean toggle to print operational debug messages.")
    args = parser.parse_args()

    rclpy.init()

    gps = GPSNode(args.debug)
    gps.run()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps.destroy_node()
    rclpy.shutdown()