
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
# from terrasentia_msgs.msg import TerraGps



class GPSNode(Node):

    def __init__(self,debug,topic):
        super().__init__('ros2_gps_node')
        """Establishing connection to the control device.  """
        print("Begining to initialize node")
        self.session = gps.gps("localhost", "2947")
        self.session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
        self.debug = debug
        self.gpsPublisher = self.create_publisher(NavSatFix, topic,1)
        print("Initialized Publisher and connection to gps")

    def run(self):
        while True:
            #Reading acceleration data
            report = self.session.next()

            if self.debug: print(report)
            if report["class"] == "TPV":
                if "time" in report:
                    gpsMSG = NavSatFix()
                    gpsMSG.latitude = report["lat"]
                    gpsMSG.longitude = report["lon"]
                    try:
                        gpsMSG.status = report["status"]
                    except:
                        pass
                    try:
                        gpsMSG.altitude = report["alt"]
                    except:
                        pass
                    current_time = modf(time.time())
                    # gpsMSG.header.sec = int(current_time[1])
                    # gpsMSG.header.nanosec = int(current_time[0] * 1000000000) & 0xffffffff
                    self.gpsPublisher.publish(gpsMSG)
                    print("Sent GPS message", report)
            else:
                if self.debug: print("GPS has not locked onto sattelites")





def main():
    #Createion of
    parser = argparse.ArgumentParser(description='Arguments for Imu Node')
    parser.add_argument("--debug",default=False,action="store_true", help="Boolean toggle to print operational debug messages.")
    parser.add_argument("-t", "--topic",default="/gps/fix", help="Topic Name")
    args = parser.parse_args()

    rclpy.init()

    gpsNode = GPSNode(args.debug,args.topic)
    gpsNode.run()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gpsNode.destroy_node()
    rclpy.shutdown()
