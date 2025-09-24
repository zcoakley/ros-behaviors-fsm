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
import matplotlib.pyplot as plt
from time import sleep


class PersonFollowerNode(Node):
    """
    A Node subclass that makes the Neato follow a person.
    """

    LASER_SCAN_RANGE_MIN = 0.01  # meters
    LASER_SCAN_RANGE_MAX = 5.0  # meters
    LASER_SCAN_ANGLE_INCREMENT = 0.017453277483582497  # radians
    LASER_SCAN_ARRAY_LENGTH = 361
    Kp_angular = 0.02
    Kp_linear = 0.5
    _distances = [0.0] * LASER_SCAN_ARRAY_LENGTH
    _filter_distance = 1
    _person_follow_point_cutoff = 1

    def __init__(self):
        super().__init__("person_follower")
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
        # print(self._distances)
        msg = Twist()
        # return if self._distances and self._angles haven't been populated yet

        [person_angle, person_distance] = self.find_person()
        # [linear_velocity, angular_velocity] = self.follow_person(
        #     person_angle, person_distance, 1
        # )
        msg.angular.z = person_angle * self.Kp_angular
        msg.linear.x = min(person_distance * self.Kp_linear, 0.3)
        print([msg.angular.z, msg.linear.x])

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
        # Count the number of valid points (points <= 2m away) in distance_array
        distance_array = [
            0 if d > self._filter_distance else d for d in self._distances
        ]
        # print(distance_array)
        person_point_angles = []
        person_point_distance = []
        for angle, distance in enumerate(distance_array):
            if distance > 0:
                person_point_angles.append(angle)
                person_point_distance.append(distance)

        if len(person_point_angles) < self._person_follow_point_cutoff:
            return 0.0, 0.0
        person_point_angles = convert_to_signed_angles(person_point_angles)

        # Actually, use the angles output by process_scan---just make sure they are in degrees
        person_angle = float(statistics.mean(person_point_angles))
        person_distance = float(statistics.mean(person_point_distance))
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
        linear_velocity = math.max(0, person_distance - target_distance * self.Kp)
        angular_velocity = person_angle * self.Kp

        return linear_velocity, angular_velocity


def convert_to_signed_angles(angle_list):
    """Convert list of angles from [0, 360) to [-180, 180)"""
    return [(angle + 180) % 360 - 180 for angle in angle_list]


def convert_to_cartesian(distances, angle_start=0, angle_step=1, degrees=True):
    x_val = []
    y_val = []
    for i, d in enumerate(distances):
        angle = angle_start + i * angle_step
        if degrees:
            angle = math.radians(angle)  # convert to radians if in degrees

        x = d * math.cos(angle)
        y = d * math.sin(angle)
        x_val.append(x)
        y_val.append(y)

    return x_val, y_val


def plot_cartesian(x_val, y_val):
    plt.figure(figsize=(6, 6))
    plt.scatter(x_val, y_val, s=5, c="blue")  # s = marker size
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.title("LiDAR Scan - Cartesian View")
    plt.axis("equal")  # Equal aspect ratio for accurate spatial representation
    plt.grid(True)
    plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()
    print("shutdown")


if __name__ == "__main__":
    main()
