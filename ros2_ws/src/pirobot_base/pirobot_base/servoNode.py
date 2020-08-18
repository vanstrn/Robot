
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import RPi.GPIO as GPIO                        #Import GPIO library

GPIO.setmode(GPIO.BOARD)                       #Set GPIO pin numbering

GPIO.setwarnings(False)

#Testing some commit changes.

class ServoNode(Node):
    ''' Class to handle interaction with the motor pins
    Supports redefinition of "forward" and "backward" depending on how motors are connected
    Use the supplied Motorshieldtest module to test the correct configuration for your project.

    Arguments:
    motor = string motor pin label (i.e. "MOTOR1","MOTOR2","MOTOR3","MOTOR4") identifying the pins to which
            the motor is connected.
    config = int defining which pins control "forward" and "backward" movement.
    '''

    def __init__(self, topic, pin, lb=0,up=180,debug=False, updatePeriod=0.1):
        super().__init__('motor_controller')
        self.subscriber = self.create_subscription(Int64, topic, self.MotorCallback,10)

        GPIO.setup(pin,GPIO.OUT)
        self.servo = GPIO.PWM(pin,50)

        self.get_logger().info("Created GPIO connection to servo.")

        timer = self.create_timer(updatePeriod , self.UpdateMotors)
        self.get_logger().info("Created timer to update motors.")

        self.max_duty = 12
        self.min_duty = 2
        self.ub=ub
        self.lb=lb

    def MotorCallback(self,data):
        if data.data > self.ub:
            self.angle = self.ub
            self.get_logger().info("Specified angle was higher than servo ub. Defaulting to upper bound.")
        elif data.data < self.lb:
            self.angle = self.ul
            self.get_logger().info("Specified angle was lower than servo lb. Defaulting to lower bound.")
        else:
        self.angle = data.data

    def UpdateMotors(self):
        ds = self.min_duty + (self.max_duty-self.min_duty)*(self.data/self.ub)
        self.servo.ChangeDutyCycle(ds)
        self.get_logger().debug("Updating servo angle to: " + str(self.data))

def main(args=None):
    rclpy.init(args=args)

    parser.add_argument("-t", "--topic", required=True, type=str,
                        help="Enable Pin")
    parser.add_argument("-p", "--pin", required=True, type=int,
                        help="Pin Number")
    parser.add_argument("--debug",default=False,action="store_true", help="Boolean toggle to print operational debug messages.")
    arg = parser.parse_args()

    node = ServoNode(   topic=arg.topic,
                        enable=arg.pin,
                        debug=arg.debug)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
