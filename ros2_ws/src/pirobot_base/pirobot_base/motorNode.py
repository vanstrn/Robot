
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import RPi.GPIO as GPIO                        #Import GPIO library
import time
from time import sleep
GPIO.setmode(GPIO.BOARD)                       #Set GPIO pin numbering

GPIO.setwarnings(False)

#Testing some commit changes.

class Motor:
    ''' Class to handle interaction with the motor pins
    Supports redefinition of "forward" and "backward" depending on how motors are connected
    Use the supplied Motorshieldtest module to test the correct configuration for your project.

    Arguments:
    motor = string motor pin label (i.e. "MOTOR1","MOTOR2","MOTOR3","MOTOR4") identifying the pins to which
            the motor is connected.
    config = int defining which pins control "forward" and "backward" movement.
    '''
    motorpins = {"MOTOR3":{"config":{1:{"e":32,"f":24,"r":26},2:{"e":32,"f":26,"r":24}},"arrow":1},
                 "MOTOR4":{"config":{1:{"e":19,"f":21,"r":23},2:{"e":19,"f":23,"r":21}}, "arrow":2},
                 "MOTOR2":{"config":{1:{"e":22,"f":16,"r":18},2:{"e":22,"f":18,"r":16}}, "arrow":3},
                 "MOTOR1":{"config":{1:{"e":11,"f":15,"r":13},2:{"e":11,"f":13,"r":15}},"arrow":4}}

    def __init__(self, motor, config):
        self.testMode = False
        self.pins = self.motorpins[motor]["config"][config]
        GPIO.setup(self.pins['e'],GPIO.OUT)
        GPIO.setup(self.pins['f'],GPIO.OUT)
        GPIO.setup(self.pins['r'],GPIO.OUT)
        self.PWM = GPIO.PWM(self.pins['e'], 50)  # 50Hz frequency
        self.PWM.start(0)
        GPIO.output(self.pins['e'],GPIO.HIGH)
        GPIO.output(self.pins['f'],GPIO.LOW)
        GPIO.output(self.pins['r'],GPIO.LOW)

    def test(self, state):
        ''' Puts the motor into test mode
        When in test mode the Arrow associated with the motor receives power on "forward"
        rather than the motor. Useful when testing your code.

        Arguments:
        state = boolean
        '''
        self.testMode = state

    def forward(self, speed):
        ''' Starts the motor turning in its configured "forward" direction.

        Arguments:
        speed = Duty Cycle Percentage from 0 to 100.
        0 - stop and 100 - maximum speed
        '''
        print("Forward")
        self.PWM.ChangeDutyCycle(speed)
        GPIO.output(self.pins['f'],GPIO.HIGH)
        GPIO.output(self.pins['r'],GPIO.LOW)

    def reverse(self,speed):
        ''' Starts the motor turning in its configured "reverse" direction.

        Arguments:
        speed = Duty Cycle Percentage from 0 to 100.
        0 - stop and 100 - maximum speed
     '''
        print("Reverse")
        self.PWM.ChangeDutyCycle(speed)
        GPIO.output(self.pins['f'],GPIO.LOW)
        GPIO.output(self.pins['r'],GPIO.HIGH)

    def stop(self):
        ''' Stops power to the motor,
     '''
        print("Stop")
        self.PWM.ChangeDutyCycle(0)
        GPIO.output(self.pins['f'],GPIO.LOW)
        GPIO.output(self.pins['r'],GPIO.LOW)

    def speed(self):
        ''' Control Speed of Motor,
     '''


class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')

        self.subscriber = self.create_subscription(Twist, "cmd_vel", self.motionCallback,10)
        self.m1 = Motor("MOTOR1",2)
        self.m2 = Motor("MOTOR2",1)

        """
        M1     Front    M2



        M3     Rear     M4
        """
    def motionCallback(self,data):
        print("Updating Vehicle Motion")
        v1 = data.linear.x
        theta1 = data.angular.z
        turnRate = 25
        forRate = 50
        if forRate*v1 + turnRate*theta1 > 0:
            self.m1.forward(forRate*v1 + turnRate*theta1)
        else:
            self.m1.reverse(abs(forRate*v1 + turnRate*theta1))
        if forRate*v1 - turnRate*theta1 > 0:
            self.m2.forward(forRate*v1 - turnRate*theta1)
        else:
            self.m2.reverse(abs(forRate*v1 - turnRate*theta1))


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MotorController()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
