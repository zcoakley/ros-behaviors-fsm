"""
This is a teleop node that lets the user control the Neato's movements using keyboard input.
"""

import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopNode(Node):
    """
    A Node subclass that lets the user control the Neato's movements using keyboard input.
    """

    def __init__(self):
        super().__init__("teleop")
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_loop(self):
        msg = Twist()

        key = None
        while key != "\x03":
            key = self.get_key()
            if key == 65:
                msg.linear.x = 0.2
            self.vel_pub.publish(msg)
            # print(key)
        sys.exit()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
