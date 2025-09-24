"""
This is a finite state machine that switches between person following and an e stop.
"""

import statistics
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
    Finite state machine that switches between person following mode and paused (e stop) mode.
    """

    # Person follower setup
    LASER_SCAN_RANGE_MIN = 0.01  # meters
    LASER_SCAN_RANGE_MAX = 5.0  # meters
    LASER_SCAN_ANGLE_INCREMENT = 0.017453277483582497  # radians
    LASER_SCAN_ARRAY_LENGTH = 361
    Kp_angular = 0.02
    Kp_linear = 0.5

    def __init__(self):
        super().__init__("fsm")
        self.create_timer(0.1, self.run_loop)
        self.state = "person_follower"

        # Person Follower setup
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(
            LaserScan, "scan", self.process_scan, qos_profile=qos_profile_sensor_data
        )
        self.create_subscription(
            Bump, "bump", self.process_bump, qos_profile=qos_profile_sensor_data
        )
        self._distances = [0.0] * self.LASER_SCAN_ARRAY_LENGTH
        self._filter_distance = 1.5
        self._person_follow_point_cutoff = 1

        self._bumped = False

        # E stop setup
        self.settings = termios.tcgetattr(sys.stdin)

    def run_loop(self):
        """
        Run either the person follower or the e_stop, depending on the current state.
        """
        if self.state == "person_follower":
            self.state = self.run_person_follower_loop()
        elif self.state == "e_stop":
            self.state = self.run_estop_loop()

    # Person follower functions
    def run_person_follower_loop(self):
        """
        Run the person following behavior.

        Publishes:
            Twist representing the desired linear and angular velocities to the /cmd_vel
            topic.

        Returns:
            str: "e_stop" or "person_follower", indicating whether to stay in the
                 person follower state or switch to the e stop state.
        """
        msg = Twist()

        [person_angle, person_distance] = self.find_person()
        msg.angular.z = person_angle * self.Kp_angular
        msg.linear.x = min(person_distance * self.Kp_linear, 0.3)
        print([msg.angular.z, msg.linear.x])
        self.vel_pub.publish(msg)

        if self._bumped is True:
            return "e_stop"
        return "person_follower"

    def process_scan(self, msg):
        """
        This function is called whenever a new lidar scan is published on the /scan
        topic. It creates a distances and angles array, which contain the distance
        from the Neato and angle (starting from the front of the Neato and going
        counterclockwise) of each point in the scan. If the resulting distance is
        out of the Neato lidar's scan range, the value becomes None.

        Args:
            msg: A LaserScan message, which is passed in by the subscription.

        Returns:
            distances: An array of floats containing the distance in meters of each
                       point in the scan.
            angles: An array of floats containing the angle (in degrees, going
                    counterclockwise from the front of the Neato) of each point in
                    the scan.
        """
        for i, distance in enumerate(msg.ranges):
            if self.LASER_SCAN_RANGE_MIN < distance < self.LASER_SCAN_RANGE_MAX:
                self._distances[i] = distance
            else:
                self._distances[i] = 0

    def find_person(self):
        """
        This function locates where a person is relative to the Neato, assuming they
        are the only large object within a radius of 2 meters.

        Args:
            distance_array: an array of floats representing the distance from the Neato to
            each point near it

        Returns:
            person_angle: a float representing the person's angle in degrees
            person_distance: a float representing the person's distance in degrees
        """
        distance_array = [
            0 if d > self._filter_distance else d for d in self._distances
        ]

        person_point_angles = []
        person_point_distance = []
        for angle, distance in enumerate(distance_array):
            if distance > 0:
                person_point_angles.append(angle)
                person_point_distance.append(distance)

        if len(person_point_angles) < self._person_follow_point_cutoff:
            return 0.0, 0.0
        person_point_angles = self.convert_to_signed_angles(person_point_angles)

        person_angle = float(statistics.mean(person_point_angles))
        person_distance = float(statistics.mean(person_point_distance))
        return person_angle, person_distance

    def convert_to_signed_angles(self, angle_list):
        """Convert list of angles from [0, 360) to [-180, 180)"""
        return [(angle + 180) % 360 - 180 for angle in angle_list]

    def process_bump(self, msg):
        """
        Register if the Neato has bumped into the person.

        Args:
            msg (Bump): passed in by the subscription.
        """
        if msg.left_front or msg.right_front:
            self._bumped = True

    # E stop functions
    def run_estop_loop(self):
        """
        Make the Neato do nothing unles the spacebar is pressed.

        Pressing spacebar will change the FSM state back to person
        following.

        Returns:
            str: the state to switch to/continue with (either "e_stop" or "person_follower")
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
            self._bumped = False
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
