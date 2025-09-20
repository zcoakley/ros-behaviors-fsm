"""
This is a wall follower node, which directs the Neato to follow a wall to its right.
"""

import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump


class FSMNode(Node):
    """
    Finite state machine that switches between person following mode and paused mode.
    """

    def __init__(self):
        super().__init__("fsm")
        self.create_timer(0.1, self.run_loop)
        self.state = "person_follower"

        self.person_follower = PersonFollowerNode()
        self.e_stop = EStopNode()

    def run_loop(self):
        """
        docstring
        """
        if self.state == "person_follower":
            self.state = self.person_follower.run_loop()
        if self.state == "e_stop":
            self.state = self.e_stop.run_loop()


class PersonFollowerNode(Node):
    """
    Copy from person follower when done.
    """

    _bumped = False

    def __init__(self):
        super().__init__("person_follower")
        self.create_subscription(
            Bump, "bump", self.process_bump, qos_profile=qos_profile_sensor_data
        )

    def run_loop(self):
        if self._bumped is True:
            return "e_stop"
        return "person_follower"

    def process_bump(self, msg):
        if msg.left_front or msg.right_front == 1:
            self._bumped = True


class EStopNode(Node):
    """
    Do nothing until the spacebar is pressed on the user's laptop.
    """

    def __init__(self):
        super().__init__("e_stop")
        self.settings = termios.tcgetattr(sys.stdin)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

    def run_loop(self):
        """
        docstring
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)

        key = self.get_key()

        if key == "\x03":
            rclpy.shutdown()
            return None

        if key == " ":
            return "person_follower"

        return "e_stop"

    def get_key(self):
        """
        Return a keystroke from the keyboard input.

        Returns:
            A str representing the pressed key, if available, else an empty string.
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
