
# Goal:
#   -Create a ROS2 Node that is able to send verbose IMU information.
#   -Have publishing options to control the publishing rate/method of the node.
#       -Publish Button presses when they are recorded
#       -Publish Axes controlles every X.XXX seconds (Reduce strain on network.)
#       -Have different key mappings depending on the controller method. These are mapped to the same published topics on each device.
#
# Based on: https://github.com/m-rtijn/mpu6050
#   https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf


import os, struct, array
from fcntl import ioctl
import argparse
import rclpy
from rclpy.node import Node
import time
from math import modf
import smbus

from sensor_msgs.msg import Imu,Temperature
from std_msgs.msg import Header



class ImuNode(Node):

    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    # Available channels
    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B

    def __init__(self,address,bus=1,debug=False):
        super().__init__('imu')
        """Establishing connection to the control device.  """
        self.debug=debug
        self.address = address
        self.bus = smbus.SMBus(bus)

        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
        self.imuPublisher = self.create_publisher(Temperature, '/imu/temp',1)
        self.tempPublisher = self.create_publisher(Imu, '/imu/imu',1)

    def read_i2c_word(self, register):
        """Read two i2c registers and combine them.
        register -- the first register to read from.
        Returns the combined read results.
        """
        # Read the data from the registers
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)

        value = (high << 8) + low

        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    def set_accel_range(self, accel_range):
        """Sets the range of the accelerometer to range.
        accel_range -- the range to set the accelerometer to. Using a
        pre-defined range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

    def read_accel_range(self, raw = False):
        """Reads the range the accelerometer is set to.
        If raw is True, it will return the raw value from the ACCEL_CONFIG
        register
        If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
        returns -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1
    def set_gyro_range(self, gyro_range):
        """Sets the range of the gyroscope to range.
        gyro_range -- the range to set the gyroscope to. Using a pre-defined
        range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

    def read_gyro_range(self, raw = False):
        """Reads the range the gyroscope is set to.
        If raw is True, it will return the raw value from the GYRO_CONFIG
        register.
        If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
        returned value is equal to -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def run(self):
        while true:
            #Reading acceleration data
            x = self.read_i2c_word(self.ACCEL_XOUT0)
            y = self.read_i2c_word(self.ACCEL_YOUT0)
            z = self.read_i2c_word(self.ACCEL_ZOUT0)

            accel_range = self.read_accel_range(True)

            if accel_range == self.ACCEL_RANGE_2G:
                accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
            elif accel_range == self.ACCEL_RANGE_4G:
                accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
            elif accel_range == self.ACCEL_RANGE_8G:
                accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
            elif accel_range == self.ACCEL_RANGE_16G:
                accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
            else:
                if self.debug: print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
                accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2

            x_rot = self.read_i2c_word(self.GYRO_XOUT0)
            y_rot = self.read_i2c_word(self.GYRO_YOUT0)
            z_rot = self.read_i2c_word(self.GYRO_ZOUT0)

            gyro_scale_modifier = None
            gyro_range = self.read_gyro_range(True)

            if gyro_range == self.GYRO_RANGE_250DEG:
                gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
            elif gyro_range == self.GYRO_RANGE_500DEG:
                gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
            elif gyro_range == self.GYRO_RANGE_1000DEG:
                gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
            elif gyro_range == self.GYRO_RANGE_2000DEG:
                gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
            else:
                if self.debug: print("Unkown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
                gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

            x_rot = x_rot / gyro_scale_modifier
            y_rot = y_rot / gyro_scale_modifier
            z_rot = z_rot / gyro_scale_modifier

            # Get the actual temperature using the formule given in the
            # MPU-6050 Register Map and Descriptions revision 4.2, page 30
            raw_temp = self.read_i2c_word(self.TEMP_OUT0)
            actual_temp = (raw_temp / 340.0) + 36.53

            current_time = modf(time.time())

            imuMSG = Imu()
            imuMSG.header.stamp.sec = int(current_time[1])
            imuMSG.header.stamp.nanosec = int(current_time[0] * 1000000000) & 0xffffffff

            imuMSG.orientation.x = x_rot
            imuMSG.orientation.y = y_rot
            imuMSG.orientation.z = z_rot

            imuMSG.linear_acceleration.x = x
            imuMSG.linear_acceleration.y = y
            imuMSG.linear_acceleration.z = z

            imuMSG.angular_velocity_covariance = [-1,-1,-1,-1,-1,-1,-1,-1,-1] #Setting to -1 to represent no measurement

            if self.debug: print("Publishing", imuMSG)
            self.imuPublisher.publish(imuMSG)

            tempMSG = Temperature()
            tempMSG.temperature=actual_temp
            tempMSG.header.sec = int(current_time[1])
            tempMSG.header.nanosec = int(current_time[0] * 1000000000) & 0xffffffff
            if self.debug: print("Publishing", tempMSG)
            self.tempPublisher.publish(tempMSG)





def main():
    #Createion of
    parser = argparse.ArgumentParser(description='Arguments for Imu Node')
    parser.add_argument("-a", "--address",type=float,default="0x68", help="I2C communication address")
    parser.add_argument("-b", "--bus",type=float,default=1, help="Databus.")
    parser.add_argument("--debug",default=False,action="store_true", help="Boolean toggle to print operational debug messages.")
    args = parser.parse_args()

    rclpy.init()

    joystick = ImuNode(args.address,args.bus,args.debug)
    joystick.run()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick.destroy_node()
    rclpy.shutdown()
