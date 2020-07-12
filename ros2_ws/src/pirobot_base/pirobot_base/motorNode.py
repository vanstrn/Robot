
import rclpy
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

    def __init__(self, topic, enable, forward, reverse, debug=False, updatePeriod=0.1):
        super().__init__('motor_controller')
        self.pins = {"e":enable,"f":forward,"r":reverse}
        self.subscriber = self.create_subscription(Int64, topic, self.MotorCallback,10)

        GPIO.setup(self.pins['e'],GPIO.OUT)
        GPIO.setup(self.pins['f'],GPIO.OUT)
        GPIO.setup(self.pins['r'],GPIO.OUT)
        self.PWM = GPIO.PWM(self.pins['e'], 0)  # 50Hz frequency
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

    parser.add_argument("-t", "--topic", required=True, type=str,
                        help="Enable Pin")
    parser.add_argument("-e", "--enable", required=True, type=int,
                        help="Enable Pin")
    parser.add_argument("-f", "--forward", required=True, type=int,
                        help="Forward Pin")
    parser.add_argument("-r", "--reverse", required=True, type=int,
                        help="Reverse Pin")
    parser.add_argument("--debug",default=False,action="store_true", help="Boolean toggle to print operational debug messages.")
    arg = parser.parse_args()

    node = MotorNode(   topic=arg.topic,
                        enable=arg.enable,
                        forward=arg.forward,
                        reverse=arg.reverse,
                        debug=arg.debug)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
