
# Goal:
#   -Create a ROS2 Node that is able to send verbose controller information.
#   -Have publishing options to control the publishing rate/method of the node.
#       -Publish Button presses when they are recorded
#       -Publish Axes controlles every X.XXX seconds (Reduce strain on network.)
#       -Have different key mappings depending on the controller method. These are mapped to the same published topics on each device.
#
# Code is based on a combination of information from:
# https://github.com/FurqanHabibi/joystick_ros2/blob/master/joystick_ros2.py
# https://gist.github.com/rdb/8864666

import os, struct, array
from fcntl import ioctl
import argparse
import rclpy
from rclpy.node import Node
import time
from math import modf

from sensor_msgs.msg import Joy
from std_msgs.msg import Header

# Iterate over the joystick devices.
print('Available devices:')

# We'll store the states here.

# These constants were borrowed from linux/input.h
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'trottle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'x',
    0x131 : 'circle',
    0x132 : 'c',
    0x133 : 'triangle',
    0x134 : 'square',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'share',
    0x13b : 'options',
    0x13c : 'ps4',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}

PS4_CODE_MAP = {
    'ABS_X': 0,
    'ABS_Y': 1,
    'ABS_RX': 2,
    'ABS_Z': 3,
    'ABS_RZ': 4,
    'ABS_RY': 5,
    'ABS_HAT0X': 6,
    'ABS_HAT0Y': 7,
    'BTN_EAST': 0,
    'BTN_C': 1,
    'BTN_SOUTH': 2,
    'BTN_NORTH': 3,
    'BTN_WEST': 4,
    'BTN_Z': 5,
    'BTN_TL2': 6,
    'BTN_TR2': 7,
    'BTN_MODE': 8,
    'BTN_SELECT': 9,
    'BTN_START':10
}

class JoystickNode(Node):
    axis_states = {}
    button_states = {}
    axis_map = []
    button_map = []
    def __init__(self,sleep_time,auto_repeat,debug=False):
        super().__init__('joystick')
        """Establishing connection to the control device.  """
        self.debug=debug

        # Open the joystick device.
        fn = '/dev/input/js0'
        if self.debug: print('Opening %s...' % fn)
        self.jsdev = open(fn, 'rb')

        # Get the device name.
        buf = array.array('B', [0] * 64)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
        js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
        if self.debug: print('Device name: %s' % js_name)

        # Get number of axes and buttons.
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a11, buf) # JSIOCGAXES
        num_axes = buf[0]

        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
        num_buttons = buf[0]

        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(self.jsdev, 0x80406a32, buf) # JSIOCGAXMAP

        for axis in buf[:num_axes]:
            axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0
        if self.debug: print(self.axis_states.values())

        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(self.jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

        for btn in buf[:num_buttons]:
            btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0

        if self.debug: print('%d axes found: %s' % (num_axes, ', '.join(self.axis_map)))
        if self.debug: print('%d buttons found: %s' % (num_buttons, ', '.join(self.button_map)))

        #Initializing The Joy Message
        self.joy = Joy()
        self.joy.header = Header()
        self.joy.header.frame_id = ''
        self.joy.axes = list(self.axis_states.values())
        self.joy.buttons = list(self.button_states.values())

        # Joy publisher
        self.publisher_ = self.create_publisher(Joy, '/joy',1)

        # logic params

        #Parameters to control message publication frequency
        self.autorepeat_rate = auto_repeat
        self.last_publish_time = 0.0
        self.coalesce_interval = auto_repeat
        self.sleep_time = sleep_time

    def publish_joy(self):
        current_time = modf(time.time())
        self.joy.header.stamp.sec = int(current_time[1])
        self.joy.header.stamp.nanosec = int(current_time[0] * 1000000000) & 0xffffffff
        self.publisher_.publish(self.joy)
        self.last_publish_time = time.time()
        if self.debug: print(self.joy)

    def run(self):

        """Main event loop, which reads controller inputs and publishes """
        print(dir(self.jsdev))
        while True:
            evbuf = self.jsdev.read(8) #This doesn't update unless a button is pressed.
            print(evbuf)
            if evbuf:
                _, value, type, number = struct.unpack('IhBB', evbuf)

                if type & 0x80:
                     print("(initial)", end="")

                #Buttons. These will be published whenever stuff happens.
                if type & 0x01:
                    button = self.button_map[number]
                    if button:
                        self.button_states[button] = value

                        self.joy.buttons =list(self.button_states.values())
                        self.publish_joy()

                if type & 0x02:
                    axis = self.axis_map[number]
                    if axis:
                        fvalue = value / 32767.0
                        self.axis_states[axis] = fvalue
                        self.joy.axes =list(self.axis_states.values())
                        if (time.time() - self.last_publish_time > self.coalesce_interval):
                            self.publish_joy()

            #Hook if constant publishing is required
            if ((self.autorepeat_rate > 0.0) and (time.time() - self.last_publish_time > 1/self.autorepeat_rate)):
                self.publish_joy()

            # sleep to decrease cpu usage
            # time.sleep(self.sleep_time)



def main():
    #Createion of
    parser = argparse.ArgumentParser(description='Arguments for Controller Node')
    parser.add_argument("-a", "--auto",type=float,default=0.01, help="Minimum rate messages will be published at [Hz]")
    parser.add_argument("-s", "--sleep",type=float,default=0.01, help="Minimum sleep time between messages [s].")
    parser.add_argument("--debug",default=False,action="store_true", help="Boolean toggle to print operational debug messages.")
    args = parser.parse_args()

    rclpy.init()

    joystick = JoystickNode(args.sleep,args.auto,args.debug)
    joystick.run()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick.destroy_node()
    rclpy.shutdown()
