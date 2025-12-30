#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

HELP_MSG = """
---------------------------
Keyboard Teleop
w : forward
s : backward
a : turn left
d : turn right
x : stop
CTRL-C to quit
---------------------------
"""

class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.2
        self.angular_speed = 0.8
        self.get_logger().info(HELP_MSG)
        self.run()

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        twist = Twist()
        try:
            while rclpy.ok():
                key = self.get_key()

                if key == 'w':
                    twist.linear.x = self.linear_speed
                    twist.angular.z = 0.0
                elif key == 's':
                    twist.linear.x = -self.linear_speed
                    twist.angular.z = 0.0
                elif key == 'a':
                    twist.angular.z = self.angular_speed
                elif key == 'd':
                    twist.angular.z = -self.angular_speed
                elif key == 'x':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                else:
                    continue

                self.publisher_.publish(twist)

        except KeyboardInterrupt:
            pass

        # Stop robot on exit
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)


def main():
    rclpy.init()
    TeleopKeyboard()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
