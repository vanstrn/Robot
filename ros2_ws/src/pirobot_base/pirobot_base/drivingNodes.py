import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64
import argparse

def clip(input,lb,ub):
    if input < lb:
        return lb
    elif input > ub:
        return ub
    else:
        return input


class TwoWheelDriving(Node):

    def __init__(self,leftWheelTopic,rightWheelTopic,turnRate,maxSpeed,debug=False):
        super().__init__('motor_controller')
        self.turnRate = turnRate
        self.maxSpeed = maxSpeed
        self.subscriber = self.create_subscription(Twist, "cmd_vel", self.MotionCallback,10)
        self.leftWheelPub = self.create_publisher(Int64, leftWheelTopic, 1)
        self.rightWheelPub = self.create_publisher(Int64, rightWheelTopic, 1)

        self.get_logger().info("Created Node Publishers and Subscribers")

        self.declare_parameter("bias",value=0.0)
        self.declare_parameter("turn_rate",value=turnRate)
        self.declare_parameter("max_speed",value=maxSpeed)

        self.get_logger().info("Created Node Parameters")


    def MotionCallback(self,data):
        v1 = data.linear.x
        theta1 = data.angular.z

        max_speed = self.get_parameter("max_speed").get_parameter_value().double_value
        turn_rate = self.get_parameter("turn_rate").get_parameter_value().double_value
        self.get_logger().debug("turnRate: "+str(turn_rate)+ "  max_speed: "+str(max_speed))

        # Calculating speed of each of the motors
        leftSpeed =  max_speed*v1 - turn_rate*theta1
        rightSpeed = max_speed*v1 + turn_rate*theta1
        self.get_logger().debug("leftSpeed: "+str(leftSpeed)+ "  rightSpeed: "+str(rightSpeed))

        #Sending messages to the motor nodes.
        lmsg = Int64()
        lmsg.data = int(clip(leftSpeed,-98,98))
        self.leftWheelPub.publish(lmsg)

        rmsg = Int64()
        rmsg.data = int(clip(rightSpeed,-98,98))
        self.rightWheelPub.publish(rmsg)
        self.get_logger().debug("Updated Driving Motors- Right Speed:"+str(rightSpeed)+" Left Speed:"+str(leftSpeed))


def Run2WheelDriving(args=None):
    parser = argparse.ArgumentParser(description='Arguments for 2 Wheel Driving Node')
    parser.add_argument("-l", "--left",type=str,default="motor1", help="Right motor topic")
    parser.add_argument("-r", "--right",type=str,default="motor2", help="Left motor topic")
    parser.add_argument("-s", "--speed",type=float,default=50.0, help="Max speed parameter")
    parser.add_argument("-t", "--turn",type=float,default=50.0, help="Turn rate parameter")
    parser.add_argument("--debug",default=False,action="store_true", help="Boolean toggle to print operational debug messages.")
    arg = parser.parse_args()

    rclpy.init(args=args)

    minimal_publisher = TwoWheelDriving(leftWheelTopic=arg.left,
                                        rightWheelTopic=arg.right,
                                        turnRate=arg.turn,
                                        maxSpeed=arg.speed,
                                        debug=arg.debug)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
