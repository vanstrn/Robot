
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
from flask import Flask, render_template, Response, request
import signal, sys
import json


app = Flask (__name__, static_url_path = '')

def XMLDataStingToDict(string):
    outDict = {}
    for keyValuePair in string.split("&"):
        key,value = keyValuePair.split("=")
        outDict[key]=value
    return outDict


# Send out the bots control page (home page)
@app.route ("/")
def index ( ):
   return render_template ('index.html', name = None)

@app.route ("/forward")
def forward ( ):
   global last_direction, run_time

   print ("Forward")
   go_forward ( )
   last_direction = 0

   # sleep 100ms + run_time
   time.sleep (0.100 + run_time)

   # If not continuous, then halt after delay
   if run_time > 0:
      last_direction = -1
      halt ( )

   return "ok"

@app.route ("/continuous")
def continuous ( ):
   global run_time

   print ("Continuous run")
   run_time = 0

   # sleep 100ms
   time.sleep (0.100)
   return "ok"

@app.route("/take_image", methods=['GET', 'POST'])
def take_image ( ):
   global run_time
   if request.method == "POST":
       data = request.data.decode("utf-8")
       print(XMLDataStingToDict(data))
       # name = request.form["name"]
       # print(name)
   print ("Taking Picture ")

   #Update some text to say image has been saved.

   return "ok"

@app.route ("/mid_run")
def mid_run ( ):
   global run_time

   print ("Mid run")
   run_time = 0.750

   # sleep 100ms
   time.sleep (0.100)
   return "ok"

@app.route ("/short_time")
def short_time ( ):
   global run_time

   print ("Short run")
   run_time = 0.300

   # sleep 100ms
   time.sleep (0.100)
   return "ok"

if __name__ == "__main__" :
   app.run (host = '0.0.0.0', debug = True)
