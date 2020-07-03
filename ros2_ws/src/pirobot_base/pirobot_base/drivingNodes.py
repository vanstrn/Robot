import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pirobot_base.msg import Motor


class TwoWheelDriving(Node):

    def __init__(self,leftWheelTopic,rightWheelTopic,turnRate,maxSpeed,debug=False):
        super().__init__('motor_controller')
        self.turnRate = turnRate
        self.maxSpeed = maxSpeed
        self.subscriber = self.create_subscription(Twist, "cmd_vel", self.motionCallback,10)
        self.leftWheelPub = self.create_publisher(Motor, leftWheelTopic, 1)
        self.rightWheelPub = self.create_publisher(Motor, leftWheelTopic, 1)
        self.declare_parameter("bias",value=0)
        self.declare_parameter("turn_rate",value=turnRate)
        self.declare_parameter("max_speed",vale=maxSpeed)


    def motionCallback(self,data):
        v1 = data.linear.x
        theta1 = data.angular.z

        max_speed = self.get_parameter("max_speed").get_parameter_value()
        turn_rate = self.get_parameter("turn_rate").get_parameter_value()

        self.get_logger().info("Updating vehicle motors.")

        # Calculating speed of each of the motors
        leftSpeed =  max_speed*v1 + turn_rate*theta1
        rightSpeed = max_speed*v1 - turn_rate*theta1

        #Sending messages to the motor nodes.
        lmsg = Motor
        if leftSpeed > 0: lmsg.forward=True
        else: lmsg.forward=False
        lmsg.speed = int(leftSpeed)
        self.leftWheelPub.publish(lmsg)

        rmsg = Motor
        if leftSpeed > 0: rmsg.forward=True
        else: rmsg.forward=False
        rmsg.speed = int(rightSpeed)
        self.rightWheelPub.publish(rmsg)


def Run2WheelDriving(args=None):
    parser = argparse.ArgumentParser(description='Arguments for 2 Wheel Driving Node')
    parser.add_argument("-l", "--right",type=str,default="motor1", help="Right motor topic")
    parser.add_argument("-r", "--left",type=str,default="motor2", help="Left motor topic")
    parser.add_argument("-s", "--speed",type=int,default=50, help="Max speed parameter")
    parser.add_argument("-t", "--turn",type=int,default=25, help="Turn rate parameter")
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
