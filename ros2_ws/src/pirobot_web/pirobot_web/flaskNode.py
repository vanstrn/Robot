
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


#!/usr/bin/env python
import cv2
from threading import Thread, Event
from flask import Flask, render_template, Response
import signal, sys

class FlaskNode(Node):

    def __init__(self,event):
        super().__init__('flask_node')

        #Initializing Publishers
        qos = qos_profile_sensor_data
        self.bridge = CvBridge()
        self.subscriber = self.create_subscription(Image, "image/raw", self._ImageCallback,qos_profile_sensor_data)
        self.event =event

    def _ImageCallback(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.frame = cv2.imencode(".jpg",cv_image)[1].tobytes()
        self.event.set()

    def GetFrame(self):
        return self.frame


    def _InitializeParameters(self):
        self.declare_parameter("frame_rate",value=60.0)

def main(args=None):
    rclpy.init(args=args)

    # parser = argparse.ArgumentParser(description='Arguments for Imu Node')
    # args = parser.parse_args()

    event = Event()
    node = FlaskNode(event)

    app = Flask(__name__)

    @app.route('/')
    def index():
        return render_template('index.html')

    def gen():
        while True:
            event.wait()
            event.clear()
            frame = node.GetFrame()
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    @app.route('/video_feed')
    def video_feed():
        return Response(gen(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')
    try:
        Thread(target=lambda: rclpy.spin(node)).start()
        app.run(host='0.0.0.0', port=8080 ,debug=True)
    except KeyboardInterrupt:
        node.get_logger().info('FLASK: Keyboard interrupt')

    node.destroy_node()

    rclpy.shutdown()
