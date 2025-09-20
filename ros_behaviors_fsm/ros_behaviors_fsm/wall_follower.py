"""
This is a wall follower node, which directs the Neato to follow a wall to its right.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollowerNode(Node):
    """
    A Node subclass that makes the Neato follow a nearby wall.

    It assumes there is a wall on the Neato's right side.
    """

    LASER_SCAN_RANGE_MIN = 0.10000000149011612  # meters
    LASER_SCAN_RANGE_MAX = 5.0  # meters
    LASER_SCAN_ANGLE_INCREMENT = 0.017453277483582497  # radians
    LASER_SCAN_ARRAY_LENGTH = 361
    Kp = 1

    def __init__(self):
        super().__init__("wall_follower")
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(
            LaserScan, "scan", self.process_scan, qos_profile=qos_profile_sensor_data
        )
        self._distances = [None] * self.LASER_SCAN_ARRAY_LENGTH
        self._angles = [None] * self.LASER_SCAN_ARRAY_LENGTH

    def run_loop(self):
        """
        Run the wall following behavior.

        This function is run on a timer every 0.1 sec. It uses proportional control to
        adjust the Neato's angular z velocity based on lider scan data from the /scan
        topic. It adjusts the velocity to turn the Neato parallel to the wall and move
        it forward at 0.2 m/s.

        Publishes:
            Twist representing the desired linear and angular velocities to the /cmd_vel
            topic.
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


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
