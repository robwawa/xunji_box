#!/usr/bin/env python3
"""键盘遥控节点 — 发布 /cmd_vel，测试 chassis_bridge_node 完整性"""

import select, sys, termios, tty
import rospy
from geometry_msgs.msg import Twist

HELP = """
=== 键盘遥控 (/cmd_vel) ===
  w/s : 前进/后退
  a/d : 左转/右转
  space : 停止
  +/- : 调线速度   ,/. : 调角速度
  q   : 退出
"""


class KeyboardVel:
    def __init__(self):
        self.lin = rospy.get_param("~linear_speed", 0.05)
        self.ang = rospy.get_param("~angular_speed", 0.15)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.v = self.w = 0.0

    def handle(self, key):
        if key == "w":       self.v, self.w = self.lin, 0.0
        elif key == "s":     self.v, self.w = -self.lin, 0.0
        elif key == "a":     self.v, self.w = 0.0, self.ang
        elif key == "d":     self.v, self.w = 0.0, -self.ang
        elif key == " ":     self.v = self.w = 0.0
        elif key in ("+", "="): self.lin = min(0.3, self.lin + 0.02)
        elif key in ("-", "_"): self.lin = max(0.02, self.lin - 0.02)
        elif key == ".":      self.ang = min(0.8, self.ang + 0.05)
        elif key == ",":      self.ang = max(0.05, self.ang - 0.05)
        elif key == "q":      return False
        else: return True
        rospy.loginfo("v=%.2f w=%.2f  speed: lin=%.2f ang=%.2f",
                      self.v, self.w, self.lin, self.ang)
        return True

    def publish(self):
        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = self.w
        self.pub.publish(msg)


def main():
    rospy.init_node("lepu_keyboard_vel")
    node = KeyboardVel()
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print(HELP)
    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r:
                key = sys.stdin.read(1).lower()
                if not node.handle(key):
                    break
            node.publish()
            rate.sleep()
    finally:
        node.v = node.w = 0.0
        node.publish()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print("\n已停止")


if __name__ == "__main__":
    main()
