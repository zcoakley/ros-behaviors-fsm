"""
This is a person follower node, which directs the Neato to follow a nearby person.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class PersonFollowerNode(Node):
    """
    A Node subclass that makes the Neato follow a person.
    """

    LASER_SCAN_RANGE_MIN = 0.10000000149011612  # meters
    LASER_SCAN_RANGE_MAX = 5.0  # meters
    LASER_SCAN_ANGLE_INCREMENT = 0.017453277483582497  # radians
    LASER_SCAN_ARRAY_LENGTH = 361
    Kp = 1
    _distances = [None] * LASER_SCAN_ARRAY_LENGTH
    _angles = [None] * LASER_SCAN_ARRAY_LENGTH

    def __init__(self):
        super().__init__("wall_follower")
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(
            LaserScan, "scan", self.process_scan, qos_profile=qos_profile_sensor_data
        )

    def run_loop(self):
        """
        Run the person following behavior.
        """
        # return if self._distances and self._angles haven't been populated yet
        if not self._distances[0]:
            return

        msg = Twist()

        # hardcoding points for now (both on the right of the neato)
        theta_1 = self._angles[250]
        theta_2 = self._angles[290]
        a = self._distances[250]
        b = self._distances[290]

        theta_c = theta_2 - theta_1

        c = math.sqrt(a**2 + b**2 - 2 * a * b * math.cos(theta_c))

        theta_b = math.asin(b / c * math.sin(theta_c))

        theta_pv = theta_1 - theta_b

        # Proportional control
        theta_out = self.Kp * theta_pv

        msg.angular.z = theta_out
        msg.linear.x = 0.2

        self.vel_pub.publish(msg)

    def process_scan(self, msg):
        """
        NOT UPDATED (might not need angles)
        Clean the Lidar scan data and calculate angles.

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
            angles: An array of floats containing the angle (in radians, going
                    counterclockwise from the front of the Neato) of each point in
                    the scan.
        """
        for i, distance in enumerate(msg.ranges):
            if self.LASER_SCAN_RANGE_MIN < distance < self.LASER_SCAN_RANGE_MAX:
                self._distances[i] = distance
                self._angles[i] = self.LASER_SCAN_ANGLE_INCREMENT * i - math.pi
            else:
                self._distances[i] = None
                self._angles[i] = None

    def find_person(distance_array):
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
        # Count the number of valid points (points < 2m away) in distance_array
        distance_array = [0 for d in distance_array if d > 2]
        num_points = len(distance_array) - distance_array.count(0)

        # Locate the median value
        num_nonzero_vals = 0
        for i in range(distance_array):
            if i != 0:
                num_nonzero_vals += 1
                if num_nonzero_vals == num_points // 2:  # fix this
                    person_angle = i
                    person_distance = i  # fix this
        return person_angle
        return person_distance


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
