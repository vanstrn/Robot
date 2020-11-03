
import rclpy
import argparse
from rclpy.node import Node
from std_msgs.msg import Int64
import RPi.GPIO as GPIO                        #Import GPIO library

GPIO.setmode(GPIO.BOARD)                       #Set GPIO pin numbering

GPIO.setwarnings(False)

#Testing some commit changes.

class MotorNode(Node):
    ''' Class to handle interaction with the motor pins
    Supports redefinition of "forward" and "backward" depending on how motors are connected
    Use the supplied Motorshieldtest module to test the correct configuration for your project.

    Arguments:
    motor = string motor pin label (i.e. "MOTOR1","MOTOR2","MOTOR3","MOTOR4") identifying the pins to which
            the motor is connected.
    config = int defining which pins control "forward" and "backward" movement.
    '''

    def __init__(self, debug=False, updatePeriod=0.1):
        super().__init__("motor")

        self.declare_parameter("enable")
        self.declare_parameter("forward")
        self.declare_parameter("reverse")
        enable = self.get_parameter("enable").get_parameter_value().integer_value
        forward = self.get_parameter("forward").get_parameter_value().integer_value
        reverse = self.get_parameter("reverse").get_parameter_value().integer_value

        if enable == 0 or forward == 0 or reverse == 0:
            self.get_logger().error("GPIO pin values have not been set. Please set parameters for 'enable', 'forward' and 'reverse'.")
            exit()

        self.pins = {"e":enable,"f":forward,"r":reverse}
        self.subscriber = self.create_subscription(Int64, "Motor1", self.MotorCallback,10)

        GPIO.setup(self.pins['e'],GPIO.OUT)
        GPIO.setup(self.pins['f'],GPIO.OUT)
        GPIO.setup(self.pins['r'],GPIO.OUT)
        self.PWM = GPIO.PWM(self.pins['e'], 50)  # 50Hz frequency
        self.PWM.start(0)
        GPIO.output(self.pins['e'],GPIO.HIGH)
        GPIO.output(self.pins['f'],GPIO.LOW)
        GPIO.output(self.pins['r'],GPIO.LOW)
        self.get_logger().info("Created GPIO connection to motor.")

        self.speed = 0

        timer = self.create_timer(updatePeriod , self.UpdateMotors)
        self.get_logger().info("Created timer to update motors.")

    def MotorCallback(self,data):
        self.speed = data.data
        self.get_logger().debug("Recieved speed change command.")

    def UpdateMotors(self):
        if self.speed == 0:
            self.PWM.ChangeDutyCycle(0)
            GPIO.output(self.pins['f'],GPIO.LOW)
            GPIO.output(self.pins['r'],GPIO.LOW)
        elif self.speed>0:
            self.PWM.ChangeDutyCycle(abs(self.speed))
            GPIO.output(self.pins['r'],GPIO.LOW)
            GPIO.output(self.pins['f'],GPIO.HIGH)
        else:
            self.PWM.ChangeDutyCycle(abs(self.speed))
            GPIO.output(self.pins['f'],GPIO.LOW)
            GPIO.output(self.pins['r'],GPIO.HIGH)

        self.get_logger().debug("Updating motor speed.")

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Arguments for Motor Node')
    parser.add_argument("--debug",default=False,action="store_true", help="Boolean toggle to print operational debug messages.")
    arg, unknown = parser.parse_known_args()

    node = MotorNode(debug=arg.debug)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
