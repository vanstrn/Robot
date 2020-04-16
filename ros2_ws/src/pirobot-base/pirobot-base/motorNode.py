
import MotorShield
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')

        self.subscriber = self.create_subscription(Twist, "cmd_vel", self.motionCallback,10)
        self.m1 = MotorShield.Motor("MOTOR1",2)
        self.m2 = MotorShield.Motor("MOTOR2",2)
        self.m3 = MotorShield.Motor("MOTOR3",1)
        self.m4 = MotorShield.Motor("MOTOR4",2)
        """
        M1     Front    M2



        M3     Rear     M4
        """
        self.ab = MotorShield.Arrow(1)
        self.al = MotorShield.Arrow(2)
        self.af = MotorShield.Arrow(3)
        self.ar = MotorShield.Arrow(4)

    def motionCallback(self,data):
        print("Updating Vehicle Motion")
        v1 = data.linear.x
        theta1 = data.angular.z
        turnRate = 25
        forRate = 50
        if forRate*v1 + turnRate*theta1 > 0:
            self.m1.forward(forRate*v1 + turnRate*theta1)
            self.m3.forward(forRate*v1 + turnRate*theta1)
        else:
            self.m1.reverse(np.abs(forRate*v1 + turnRate*theta1))
            self.m3.reverse(np.abs(forRate*v1 + turnRate*theta1))
        if forRate*v1 - turnRate*theta1 > 0:
            self.m2.forward(forRate*v1 - turnRate*theta1)
            self.m4.forward(forRate*v1 - turnRate*theta1)
        else:
            self.m2.reverse(np.abs(forRate*v1 - turnRate*theta1))
            self.m4.reverse(np.abs(forRate*v1 - turnRate*theta1))



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
