

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from sensor_msgs.msg import Joy

class ServoCommandNode(Node):

    def __init__(self):
        super().__init__('cont')

        self.subscriber = self.create_subscription(Joy, '/joy', self.motionCallback,10)
        self.publisher = self.create_publisher(Int64, '/servo', 1)
        print("Setup Finished")

    def motionCallback(self,data):
        # print("Updating Vehicle Motion")
        self.axes = data.axes
        self.buttons = data.buttons

        servoMsg = Twist()
        servoMsg.data = self.axes[2]*90

        if self.buttons[2]==1:
            print(servoMsg.data)

        self.publisher.publish(servoMsg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ServoCommandNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
