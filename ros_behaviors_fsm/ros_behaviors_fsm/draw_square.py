"""
Draw Square
--------
This node encapsulates implements a simple time-based approach to driving the
robot in a square.
"""

import rclpy
from rclpy.node import Node
from threading import Thread, Event
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math


class DrawSquare(Node):
    """
    Class that gets robot to drive in square

    Attributes:
        vel_pub (rclpy.publisher.Publisher): Published for velocities
        run_loop_thread (Thread): Thread that runs drive loop
    """

    def __init__(self):
        """
        Initializes DrawSquare node
        """
        super().__init__("draw_square")
        # create a thread to handle long-running component
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.run_loop_thread = Thread(target=self.run_loop)
        self.run_loop_thread.start()

    def run_loop(self):
        """
        Main drive loop to get robot to drive in a square
        """
        # the first message on the publisher is often missed
        self.drive(0.0, 0.0)
        sleep(1)
        for _ in range(4):
            print("driving forward")
            self.drive_forward(0.5)
            print("turning left")
            self.turn_left()
        print("done with run loop")

    def drive(self, linear, angular):
        """
        Drive with given linear and angular velocity

        Args:
            linear (float): Linear velocity in m/s
            angular (float): Angular velocity in radians/s
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def turn_left(self):
        """
        Execute a 90 degree left turn
        """
        angular_vel = 0.3
        self.drive(linear=0.0, angular=angular_vel)
        sleep(math.pi / angular_vel / 2)
        self.drive(linear=0.0, angular=0.0)

    def drive_forward(self, distance):
        """
        Drive straight for given distance

        Args:
            distance (float): Distance to drive forward. Only positive values
        """
        forward_vel = 0.1
        self.drive(linear=forward_vel, angular=0.0)
        sleep(distance / forward_vel)
        self.drive(linear=0.0, angular=0.0)


def main(args=None):
    rclpy.init(args=args)
    node = DrawSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
