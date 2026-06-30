#!/usr/bin/env python3
"""Keyboard teleop publisher for Lepu car cmd_vel control (ROS1)."""

import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


HELP = """
=== Lepu Keyboard Control (/cmd_vel) ===
  w / x     forward / backward
  a / d     turn left / turn right
  q / e     forward-left / forward-right
  z / c     backward-left / backward-right
  s / space stop
  + / -     adjust linear speed
  , / .     adjust angular speed
  Ctrl-C    quit
"""


class LepuKeyboardTeleop(object):
    def __init__(self):
        self.linear_speed = rospy.get_param('~linear_speed', 0.08)
        self.angular_speed = rospy.get_param('~angular_speed', 0.25)
        publish_rate_hz = rospy.get_param('~publish_rate_hz', 20.0)
        cmd_vel_topic = rospy.get_param('~cmd_vel_topic', 'cmd_vel')

        self._linear = 0.0
        self._angular = 0.0
        self._pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self._rate = rospy.Rate(publish_rate_hz)

    def handle_key(self, key):
        if key == 'w':
            self._linear = self.linear_speed
            self._angular = 0.0
            self._print_cmd()
        elif key == 'x':
            self._linear = -self.linear_speed
            self._angular = 0.0
            self._print_cmd()
        elif key == 'a':
            self._linear = 0.0
            self._angular = self.angular_speed
            self._print_cmd()
        elif key == 'd':
            self._linear = 0.0
            self._angular = -self.angular_speed
            self._print_cmd()
        elif key == 'q':
            self._linear = self.linear_speed
            self._angular = self.angular_speed
            self._print_cmd()
        elif key == 'e':
            self._linear = self.linear_speed
            self._angular = -self.angular_speed
            self._print_cmd()
        elif key == 'z':
            self._linear = -self.linear_speed
            self._angular = -self.angular_speed
            self._print_cmd()
        elif key == 'c':
            self._linear = -self.linear_speed
            self._angular = self.angular_speed
            self._print_cmd()
        elif key in {'s', ' '}:
            self.stop()
            self._print_cmd()
        elif key in {'+', '='}:
            self.linear_speed = min(self.linear_speed + 0.02, 0.30)
            self._print_speed()
        elif key in {'-', '_'}:
            self.linear_speed = max(self.linear_speed - 0.02, 0.02)
            self._print_speed()
        elif key == '.':
            self.angular_speed = min(self.angular_speed + 0.05, 0.80)
            self._print_speed()
        elif key == ',':
            self.angular_speed = max(self.angular_speed - 0.05, 0.05)
            self._print_speed()

    def stop(self):
        self._linear = 0.0
        self._angular = 0.0
        self._publish()

    def _publish(self):
        msg = Twist()
        msg.linear.x = self._linear
        msg.angular.z = self._angular
        self._pub.publish(msg)

    def _print_speed(self):
        rospy.loginfo(
            'Speed: linear=%.2f m/s, angular=%.2f rad/s',
            self.linear_speed, self.angular_speed,
        )

    def _print_cmd(self):
        rospy.loginfo('cmd_vel: linear=%.2f, angular=%.2f', self._linear, self._angular)

    def spin(self):
        """Publish continuously and handle keys in a loop."""
        while not rospy.is_shutdown():
            self._publish()
            self._rate.sleep()


def read_key(timeout_sec=0.1):
    readable, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if not readable:
        return ''
    return sys.stdin.read(1)


def main():
    rospy.init_node('lepu_keyboard_teleop')
    node = LepuKeyboardTeleop()
    old_settings = termios.tcgetattr(sys.stdin)

    print(HELP)
    node._print_speed()

    try:
        tty.setcbreak(sys.stdin.fileno())
        while not rospy.is_shutdown():
            key = read_key(0.05)
            if key == '\x03':
                break
            if key:
                node.handle_key(key.lower())
            node._publish()
            rospy.sleep(0.05)
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == '__main__':
    main()
