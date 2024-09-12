"""This script is for publish ROS messages in Python."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped


class SendMessageNode(Node):
    """This is a message sending node which inherits from Node."""

    def __init__(self):
        """Initializes the class"""
        super().__init__("send_message_node")
        # Create a timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(PointStamped, "my_point", 10)

    def run_loop(self):
        """Prints a message to the terminal."""
        my_header = Header(stamp=self.get_clock().now().to_msg(), frame_id="odom")
        my_point = Point(x=1.0, y=2.0, z=3.0)
        my_point_stamped = PointStamped(header=my_header, point=my_point)
        print(my_point_stamped)
        self.publisher.publish(my_point_stamped)


def main(args=None):
    """Initialize our node, run it, cleanup on shut down"""
    rclpy.init(args=args)
    node = SendMessageNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
