import rclpy
from rclpy.node import Node
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist


class EmergencyStopNode(Node):
    """This node stops the the motors when bump is triggered"""

    def __init__(self):
        """Initializes the class."""
        super().__init__("emergencency_stop_node")
        self.create_timer(0.1, self.run_loop)  # timer runs the loop
        self.create_subscription(
            Bump, "bump", self.process_bump, 10
        )  # continually broadcasts messages
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)  #
        self.bumper_active = False

        # create a timer that runs the robot motors
        # create a state that stores the bumper information
        # create a subscriber to the bumper information
        # create a subscriber to the bump sensor
        # create a publisher for the motors

    def process_bump(self, msg):
        """callback from handling a bump sensor input"""
        # process message
        # determine if a bump has happened
        # set the bumper state variable
        self.bumper_active = (
            msg.left_front == 1
            or msg.left_side == 1
            or msg.right_front == 1
            or msg.right_side == 1
        )

    def run_loop(self):
        """Event loop drives the motors based on the bumper"""
        msg = Twist()
        msg.linear.x = 0.1 if not self.bumper_active else 0.0
        self.vel_pub.publish(msg)
        # command the robot to move
        # if a bump is sensed, stop robot motion


def main(args=None):
    """Initialization, Spin, Shutdown"""
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
