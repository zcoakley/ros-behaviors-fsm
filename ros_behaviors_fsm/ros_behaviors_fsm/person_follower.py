"""
This is a person follower node, which directs the Neato to follow a nearby person.
"""

import math
import statistics
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

        Publishes: 
            Twist representing the desired linear and angular velocities to the /cmd_vel
            topic.
        """
        # return if self._distances and self._angles haven't been populated yet
        if not self._distances[0]:
            return

        msg = Twist()

        [person_angle, person_distance] = self.find_person()
        [linear_velocity, angular_velocity] = self.follow_person(person_angle, person_distance, 1)
        msg.angular.z = angular_velocity
        msg.linear.x = linear_velocity

        self.vel_pub.publish(msg)

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
                self._angles[i] = i - 180
            else:
                self._distances[i] = None
                self._angles[i] = None

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
        # Count the number of valid points (points <= 2m away) in distance_array
        distance_array = [0 for d in self._distances if d > 2]
        point_distances = [d for d in distance_array if d != 0]
        point_angles = [d.index() for d in distance_array if d != 0] # Test to make sure index() works as expected
        # Actually, use the angles output by process_scan---just make sure they are in degrees

        person_angle = statistics.median(point_angles)
        person_distance = statistics.median(point_distances)

        return person_angle, person_distance

    def follow_person(self, person_angle, person_distance, target_distance):
        """
        This function enables the Neato to maintain a fixed distance behind a person by taking a person's location
        as input and outputting the correct linear and angular velocities.

        Args:
            person_angle: a float representing the person's angle in degrees
            person_distance: a float representing the person's distance in degrees
            target_distance: a float representing the ideal distance between the person and Neato

        Returns:
            linear_velocity: a float representing the linear velocity to which the Neato's wheels should be set
            angular_velocity: a float representing an angular velocity to which the Neato's wheels should be set
        """

        if person_angle > 180:
            person_angle = 0 - (360 - person_angle)
        linear_velocity = math.max(0, person_distance-target_distance*self.Kp)
        angular_velocity = person_angle*self.Kp

        return linear_velocity, angular_velocity

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
