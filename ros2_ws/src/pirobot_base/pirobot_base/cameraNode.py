
import queue
import threading
import time
import sys
import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import cv2

from cv_bridge import CvBridge

from rclpy.qos import qos_profile_sensor_data

class USBCameraNode(Node):

    def __init__(self):
        super().__init__('usb_camera_node')

        #Initializing Camera
        self._InitializeParameters()
        self.camera = cv2.VideoCapture(self.get_parameter("camera_id").get_parameter_value().integer_value)
        self._InitializeCamera()
        #Initializing Publishers
        qos = qos_profile_sensor_data
        self.publisher = self.create_publisher(Image,"image/raw",qos)
        self.publisher_compressed = self.create_publisher(CompressedImage,"image/compressed",qos)
        self.bridge = CvBridge()
        self.frame_num=0

        while(self.camera.isOpened()):
            # Capture frame-by-frame
            ret, frame = self.camera.read()
            if ret == True:
                self.frame_num += 1

                if self.get_parameter("raw_publisher").get_parameter_value().bool_value:
                    msg = self.bridge.cv2_to_imgmsg(self.frame, encoding="passthrough")
                    msg.header.frame_id = str(self.frame_num)
                    self.publisher.publish(msg)

                #Publishing a compressed image representation.
                if self.get_parameter("compressed_publisher").get_parameter_value().bool_value:
                    msg = self.bridge.cv2_to_compressed_imgmsg(self.frame)
                    msg.header.frame_id = str(self.frame_num)
                    self.publisher_compressed.publish(msg)

    def _InitializeCamera(self):
        ret = self.camera.set(cv2.CAP_PROP_FPS,self.get_parameter("frame_rate").get_parameter_value().double_value)
        if self.camera.get(cv2.CAP_PROP_FPS) != self.get_parameter("frame_rate").get_parameter_value().double_value:
            self.get_logger().warning("Failed to set framerate to {} fps. FPS is set at {}. Check whether the camera supports this.".format(self.get_parameter("frame_rate").get_parameter_value().double_value,self.camera.get(cv2.CAP_PROP_FPS)))
        else:
            self.get_logger().info("Framerate set to {}.".format(self.camera.get(cv2.CAP_PROP_FPS)))

        ret = self.camera.set(cv2.CAP_PROP_FRAME_WIDTH,self.get_parameter("width").get_parameter_value().integer_value)
        if self.camera.get(cv2.CAP_PROP_FRAME_WIDTH) != self.get_parameter("width").get_parameter_value().integer_value:
            self.get_logger().warning("Failed to set framerate to {} fps. FPS is set at {}. Check whether the camera supports this.".format(self.get_parameter("width").get_parameter_value().integer_value,self.camera.get(cv2.CAP_PROP_FPS)))
        else:
            self.get_logger().info("Image width set to {}.".format(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)))

        ret = self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT,self.get_parameter("height").get_parameter_value().integer_value)
        if self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT) != self.get_parameter("height").get_parameter_value().integer_value:
            self.get_logger().warning("Failed to set framerate to {} fps. FPS is set at {}. Check whether the camera supports this.".format(self.get_parameter("height").get_parameter_value().integer_value,self.camera.get(cv2.CAP_PROP_FPS)))
        else:
            self.get_logger().info("Image height set to {}.".format(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    def _InitializeParameters(self):
        self.declare_parameter("frame_rate",value=60.0)
        self.declare_parameter("camera_id",value=-1)
        self.declare_parameter("width",value=640)
        self.declare_parameter("height",value=480)
        self.declare_parameter("flip_horizontal",value=False)
        self.declare_parameter("vertical",value=False)
        self.declare_parameter("raw_publisher",value=True)
        self.declare_parameter("compressed_publisher",value=True)




def main(args=None):
    rclpy.init(args=args)

    # parser = argparse.ArgumentParser(description='Arguments for Imu Node')
    # args = parser.parse_args()

    camNode = USBCameraNode()

    try:
        rclpy.spin(camNode)
    except KeyboardInterrupt:
        camNode.get_logger().info('CAM: Keyboard interrupt')

    camNode.stop_workers()

    camNode.destroy_node()

    rclpy.shutdown()

class USBThreadedCameraNode(Node):

    def __init__(self):
        super().__init__('usb_camera_node')

        #Initializing Camera
        self._InitializeParameters()
        self.camera = cv2.VideoCapture(self.get_parameter("camera_id").get_parameter_value().integer_value)
        self._InitializeCamera()
        #Initializing Publishers
        qos = qos_profile_sensor_data
        self.publisher = self.create_publisher(Image,"image/raw",qos)
        self.publisher_compressed = self.create_publisher(CompressedImage,"image/compressed",qos)
        self.bridge = CvBridge()
        self.frame_num=0

        thread = threading.Thread(target=self._Capture,args=())
        thread.daemon = True
        thread.start()

        timer = self.create_timer(0.01 , self._Publish)

        self.ready = False
        self.frame = None
        self.lastPub = 0.0
        print("Finished Initializing")


    def _Capture(self):
        while(self.camera.isOpened()):
            # Capture frame-by-frame
            self.ready, self.frame = self.camera.read()

    def _Publish(self):
        if self.ready == True and (time.time()-self.lastPub)>=1/self.get_parameter("frame_rate").get_parameter_value().double_value:
            self.frame_num += 1

            #Publishing the raw image
            if self.get_parameter("raw_publisher").get_parameter_value().bool_value:
                msg = self.bridge.cv2_to_imgmsg(self.frame, encoding="passthrough")
                msg.header.frame_id = str(self.frame_num)
                self.publisher.publish(msg)

            #Publishing a compressed image representation.
            if self.get_parameter("compressed_publisher").get_parameter_value().bool_value:
                msg = self.bridge.cv2_to_compressed_imgmsg(self.frame)
                msg.header.frame_id = str(self.frame_num)
                self.publisher_compressed.publish(msg)

            self.lastPub = time.time()

    def _InitializeCamera(self):
        ret = self.camera.set(cv2.CAP_PROP_FPS,self.get_parameter("frame_rate").get_parameter_value().double_value)
        if self.camera.get(cv2.CAP_PROP_FPS) != self.get_parameter("frame_rate").get_parameter_value().double_value:
            self.get_logger().warning("Failed to set framerate to {} fps. FPS is set at {}. Check whether the camera supports this.".format(self.get_parameter("frame_rate").get_parameter_value().double_value,self.camera.get(cv2.CAP_PROP_FPS)))
        else:
            self.get_logger().info("Framerate set to {}.".format(self.camera.get(cv2.CAP_PROP_FPS)))

        ret = self.camera.set(cv2.CAP_PROP_FRAME_WIDTH,self.get_parameter("width").get_parameter_value().integer_value)
        if self.camera.get(cv2.CAP_PROP_FRAME_WIDTH) != self.get_parameter("width").get_parameter_value().integer_value:
            self.get_logger().warning("Failed to set framerate to {} fps. FPS is set at {}. Check whether the camera supports this.".format(self.get_parameter("width").get_parameter_value().integer_value,self.camera.get(cv2.CAP_PROP_FPS)))
        else:
            self.get_logger().info("Image width set to {}.".format(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)))

        ret = self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT,self.get_parameter("height").get_parameter_value().integer_value)
        if self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT) != self.get_parameter("height").get_parameter_value().integer_value:
            self.get_logger().warning("Failed to set framerate to {} fps. FPS is set at {}. Check whether the camera supports this.".format(self.get_parameter("height").get_parameter_value().integer_value,self.camera.get(cv2.CAP_PROP_FPS)))
        else:
            self.get_logger().info("Image height set to {}.".format(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    def _InitializeParameters(self):
        self.declare_parameter("frame_rate",value=1.0)
        self.declare_parameter("camera_id",value=-1)
        self.declare_parameter("width",value=640)
        self.declare_parameter("height",value=480)
        self.declare_parameter("flip_horizontal",value=False)
        self.declare_parameter("vertical",value=False)
        self.declare_parameter("raw_publisher",value=True)
        self.declare_parameter("compressed_publisher",value=True)




def main2(args=None):
    rclpy.init(args=args)

    # parser = argparse.ArgumentParser(description='Arguments for Imu Node')
    # args = parser.parse_args()

    camNode = USBThreadedCameraNode()

    try:
        rclpy.spin(camNode)
    except KeyboardInterrupt:
        camNode.get_logger().info('CAM: Keyboard interrupt')

    camNode.stop_workers()

    camNode.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
