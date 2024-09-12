"""This script is for publish ROS messages in Python."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped


class ReceiveMessageNode(Node):
    """This is a message sending node which inherits from Node."""

    def __init__(self):
        """Initializes the class"""
        super().__init__("receive_message_node")
        self.sub = self.create_subscription(
            PointStamped, "my_point", self.processed_point, 10
        )

    def processed_point(self, msg):
        """Prints a message to the terminal."""
        print(msg.header)


def main(args=None):
    """Initialize our node, run it, cleanup on shut down"""
    rclpy.init(args=args)
    node = ReceiveMessageNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
