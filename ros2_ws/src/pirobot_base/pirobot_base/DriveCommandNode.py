

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class DriveCommandNode(Node):

    def __init__(self):
        super().__init__('cont')

        self.subscriber = self.create_subscription(Joy, '/joy', self.motionCallback,10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        print("Setup Finished")


    def motionCallback(self,data):
        # print("Updating Vehicle Motion")
        self.axes = data.axes
        self.buttons = data.buttons

        #print(self.axes)
        #print(self.buttons)

        twistMsg = Twist()
        twistMsg.linear.x = -self.axes[4]
        twistMsg.linear.y = 0.0
        twistMsg.linear.z = 0.0

        twistMsg.angular.x = 0.0
        twistMsg.angular.y = 0.0
        twistMsg.angular.z = self.axes[3]

        if self.buttons[2]==1:
            print(twistMsg.linear.x, twistMsg.angular.z)

        self.publisher.publish(twistMsg)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = DriveCommandNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
