#!/usr/bin/env python3
"""ROS1 odometry + cmd_vel node for Lepu car via SLAM 3.0 serial API."""

import math
import os
import sys
import threading

import rospy
import tf
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from lepu_car_protocol import (  # noqa: E402
    LepuSerialLink,
    parse_base_vel,
    parse_nav_pose,
    parse_wheel_encoders,
)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_quaternion(yaw):
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


class LepuChassisNode(object):
    def __init__(self):
        # ---- 参数（带 ~ 前缀表示私有参数） ----
        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baudrate = int(rospy.get_param('~baudrate', 115200))
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.odom_topic = rospy.get_param('~odom_topic', 'odom')
        self.publish_tf = bool(rospy.get_param('~publish_tf', True))
        self.heartbeat_interval = float(rospy.get_param('~heartbeat_interval_sec', 5.0))
        self.enable_pose_report = bool(rospy.get_param('~enable_pose_report', True))
        self.wheel_separation = float(rospy.get_param('~wheel_separation', 0.242))
        self.wheel_radius = float(rospy.get_param('~wheel_radius', 0.0705))
        self.encoder_ticks_per_rev = int(rospy.get_param('~encoder_ticks_per_rev', 16384))
        self.use_encoder_odom = bool(rospy.get_param('~use_encoder_odom', True))
        self.odom_source = rospy.get_param('~odom_source', 'encoder')
        self.odom_linear_scale = float(rospy.get_param('~odom_linear_scale', 1.0))
        self.odom_angular_scale = float(rospy.get_param('~odom_angular_scale', 1.0))
        self.pose_source_timeout = float(rospy.get_param('~pose_source_timeout_sec', 1.0))
        self.encoder_stale_timeout = float(rospy.get_param('~encoder_stale_timeout_sec', 0.5))
        self.derive_twist_from_pose = bool(rospy.get_param('~derive_twist_from_pose', True))
        self.enable_cmd_vel_control = bool(rospy.get_param('~enable_cmd_vel_control', True))
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', 'cmd_vel')
        self.cmd_vel_timeout = float(rospy.get_param('~cmd_vel_timeout_sec', 0.2))
        self.cmd_vel_publish_period = float(rospy.get_param('~cmd_vel_publish_period_sec', 0.1))
        self.max_linear_speed = float(rospy.get_param('~max_linear_speed', 0.2))
        self.max_angular_speed = float(rospy.get_param('~max_angular_speed', 0.6))

        self.meters_per_tick = (
            (2.0 * math.pi * self.wheel_radius) / max(self.encoder_ticks_per_rev, 1)
        )

        # ---- 内部状态 ----
        self._state_lock = threading.Lock()
        self._serial = None
        self._last_messages = []

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._linear_vel = 0.0
        self._angular_vel = 0.0

        self._last_left_encoder = None
        self._last_right_encoder = None
        self._pose_origin = None
        self._last_map_pose = None
        self._last_pose_time = rospy.Time.now()
        self._last_encoder_time = rospy.Time.now()
        self._last_base_vel_time = None
        self._last_publish_time = rospy.Time.now()
        self._last_publish_x = 0.0
        self._last_publish_y = 0.0
        self._last_publish_yaw = 0.0
        self._last_cmd_vel_time = rospy.Time.now()
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._last_sent_linear = 0.0
        self._last_sent_angular = 0.0

        # ---- ROS 接口 ----
        self._odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self._tf_broadcaster = tf.TransformBroadcaster()

        rospy.Timer(rospy.Duration(self.heartbeat_interval), self._send_heartbeat)
        rospy.Timer(rospy.Duration(0.05), self._publish_odometry)

        if self.enable_cmd_vel_control:
            self._cmd_vel_sub = rospy.Subscriber(
                self.cmd_vel_topic, Twist, self._handle_cmd_vel
            )
            rospy.Timer(
                rospy.Duration(self.cmd_vel_publish_period), self._send_cmd_vel
            )

        rospy.on_shutdown(self._on_shutdown)

        # ---- 连接串口 ----
        self._connect_serial()

    def _connect_serial(self):
        self._serial = LepuSerialLink(
            self.port,
            self.baudrate,
            on_message=self._handle_message,
        )
        try:
            self._serial.open()
        except Exception as exc:
            rospy.logerr('Failed to open serial port %s: %s', self.port, exc)
            raise

        rospy.loginfo(
            'Opened car serial %s @ %d, '
            'publishing %s -> %s, odom_source=%s',
            self.port, self.baudrate,
            self.odom_frame, self.base_frame, self.odom_source,
        )

        self._serial.send_command('model:request')
        if self.enable_pose_report:
            self._serial.send_command('nav:get_pose[open?on]')

        if self.enable_cmd_vel_control:
            rospy.loginfo(
                'Subscribing %s and sending app_vel at %.1f Hz',
                self.cmd_vel_topic,
                1.0 / max(self.cmd_vel_publish_period, 1e-6),
            )

    def _on_shutdown(self):
        if self._serial is not None:
            try:
                self._serial.send_command('app_vel[0,0]')
            except Exception:
                pass
            self._serial.close()

    def _handle_message(self, message):
        with self._state_lock:
            self._last_messages.append(message)
            if len(self._last_messages) > 50:
                self._last_messages = self._last_messages[-50:]

        if 'model:' in message or 'hfls_version:' in message:
            rospy.loginfo('Car response: %s', message)
        elif message.startswith('nav:pose:notfound'):
            rospy.logwarn('Car is still localizing (nav:pose:notfound)')

        now = rospy.Time.now()

        base_vel = parse_base_vel(message)
        if base_vel is not None and self.odom_source == 'base_vel':
            self._update_from_base_vel(base_vel[0], base_vel[1], now)

        encoders = parse_wheel_encoders(message)
        if encoders is not None and self.use_encoder_odom and self.odom_source == 'encoder':
            self._update_from_encoders(encoders[0], encoders[1], now)

        nav_pose = parse_nav_pose(message)
        if nav_pose is not None:
            x, y, yaw, _stamp = nav_pose
            with self._state_lock:
                self._last_map_pose = (x, y, yaw)
                self._last_pose_time = now
            if self.odom_source == 'nav_pose' or not self.use_encoder_odom:
                self._update_from_map_pose(x, y, yaw)

    def _update_from_base_vel(self, linear, angular, stamp):
        linear *= self.odom_linear_scale
        angular *= self.odom_angular_scale

        with self._state_lock:
            self._linear_vel = linear
            self._angular_vel = angular

        if self._last_base_vel_time is None:
            self._last_base_vel_time = stamp
            return

        dt = (stamp - self._last_base_vel_time).to_sec()
        self._last_base_vel_time = stamp
        if dt <= 0.0 or dt > 0.5:
            return

        self._integrate_motion(linear * dt, angular * dt, stamp, dt)

    def _encoder_delta(self, current, previous):
        delta = current - previous
        half_rev = self.encoder_ticks_per_rev // 2
        if delta > half_rev:
            delta -= self.encoder_ticks_per_rev
        elif delta < -half_rev:
            delta += self.encoder_ticks_per_rev
        return delta

    def _update_from_encoders(self, left_enc, right_enc, stamp):
        if self._last_left_encoder is None or self._last_right_encoder is None:
            self._last_left_encoder = left_enc
            self._last_right_encoder = right_enc
            self._last_encoder_time = stamp
            rospy.loginfo('Encoder odom initialized: L=%d, R=%d', left_enc, right_enc)
            return

        delta_left = self._encoder_delta(left_enc, self._last_left_encoder)
        delta_right = self._encoder_delta(right_enc, self._last_right_encoder)
        delta_left *= self.meters_per_tick
        delta_right *= self.meters_per_tick
        self._last_left_encoder = left_enc
        self._last_right_encoder = right_enc

        dt = (stamp - self._last_encoder_time).to_sec()
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05

        delta_center = 0.5 * (delta_left + delta_right) * self.odom_linear_scale
        delta_yaw = ((delta_right - delta_left) / self.wheel_separation) * self.odom_angular_scale

        self._integrate_motion(delta_center, delta_yaw, stamp, dt)

    def _update_from_map_pose(self, x, y, yaw):
        if self._pose_origin is None:
            self._pose_origin = (x, y, yaw)
            self._odom_x = 0.0
            self._odom_y = 0.0
            self._odom_yaw = 0.0
            self._last_map_pose = (x, y, yaw)
            return

        origin_x, origin_y, origin_yaw = self._pose_origin
        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)
        rel_x = x - origin_x
        rel_y = y - origin_y
        self._odom_x = rel_x * cos_yaw + rel_y * sin_yaw
        self._odom_y = -rel_x * sin_yaw + rel_y * cos_yaw
        self._odom_yaw = normalize_angle(yaw - origin_yaw)

    def _integrate_motion(self, delta_center, delta_yaw, stamp, dt):
        if abs(delta_center) < 1e-9 and abs(delta_yaw) < 1e-9:
            self._last_encoder_time = stamp
            self._linear_vel = 0.0
            self._angular_vel = 0.0
            return

        mid_yaw = self._odom_yaw + 0.5 * delta_yaw
        self._odom_x += delta_center * math.cos(mid_yaw)
        self._odom_y += delta_center * math.sin(mid_yaw)
        self._odom_yaw = normalize_angle(self._odom_yaw + delta_yaw)
        self._last_encoder_time = stamp

        dt = max(dt, 1e-6)
        self._linear_vel = delta_center / dt
        self._angular_vel = delta_yaw / dt

    def _derive_twist_from_pose_delta(self, x, y, yaw, dt):
        if dt <= 1e-6:
            return 0.0, 0.0

        delta_x = x - self._last_publish_x
        delta_y = y - self._last_publish_y
        delta_yaw = normalize_angle(yaw - self._last_publish_yaw)
        linear_vel = math.hypot(delta_x, delta_y) / dt
        angular_vel = delta_yaw / dt
        return linear_vel, angular_vel

    def _send_heartbeat(self, event):
        if self._serial is None or not self._serial.is_open():
            return
        try:
            self._serial.send_command('keep_connect')
        except Exception as exc:
            rospy.logwarn('Heartbeat failed: %s', exc)

    def _handle_cmd_vel(self, msg):
        linear = max(-self.max_linear_speed, min(self.max_linear_speed, msg.linear.x))
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, msg.angular.z))
        with self._state_lock:
            self._cmd_linear = linear
            self._cmd_angular = angular
            self._last_cmd_vel_time = rospy.Time.now()

    def _send_cmd_vel(self, event):
        if self._serial is None or not self._serial.is_open():
            return

        now = rospy.Time.now()
        with self._state_lock:
            cmd_age = (now - self._last_cmd_vel_time).to_sec()
            if cmd_age <= self.cmd_vel_timeout:
                linear = self._cmd_linear
                angular = self._cmd_angular
            else:
                linear = 0.0
                angular = 0.0

        if (
            abs(linear) < 1e-6
            and abs(angular) < 1e-6
            and abs(self._last_sent_linear) < 1e-6
            and abs(self._last_sent_angular) < 1e-6
        ):
            return

        try:
            self._serial.send_command('app_vel[{:.3f},{:.3f}]'.format(linear, angular))
            self._last_sent_linear = linear
            self._last_sent_angular = angular
        except Exception as exc:
            rospy.logwarn('cmd_vel send failed: %s', exc)

    def _publish_odometry(self, event):
        now = rospy.Time.now()
        with self._state_lock:
            x = self._odom_x
            y = self._odom_y
            yaw = self._odom_yaw
            linear_vel = self._linear_vel
            angular_vel = self._angular_vel
            pose_age = (now - self._last_pose_time).to_sec()
            encoder_age = (now - self._last_encoder_time).to_sec()

        if self.use_encoder_odom and encoder_age > self.pose_source_timeout:
            if self._last_map_pose is not None and pose_age <= self.pose_source_timeout:
                self._update_from_map_pose(*self._last_map_pose)
                with self._state_lock:
                    x = self._odom_x
                    y = self._odom_y
                    yaw = self._odom_yaw

        publish_dt = (now - self._last_publish_time).to_sec()
        if self.odom_source == 'encoder':
            if encoder_age > self.encoder_stale_timeout:
                linear_vel = 0.0
                angular_vel = 0.0
            elif self.derive_twist_from_pose:
                linear_vel, angular_vel = self._derive_twist_from_pose_delta(
                    x, y, yaw, publish_dt
                )

        stamp = now
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(yaw)
        odom.twist.twist.linear = Vector3(x=linear_vel)
        odom.twist.twist.angular = Vector3(z=angular_vel)

        pose_cov = [0.0] * 36
        pose_cov[0] = 0.05
        pose_cov[7] = 0.05
        pose_cov[35] = 0.1
        odom.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0] = 0.05
        twist_cov[35] = 0.05
        odom.twist.covariance = twist_cov

        self._odom_pub.publish(odom)

        if self.publish_tf:
            self._tf_broadcaster.sendTransform(
                (x, y, 0.0),
                yaw_to_quaternion(yaw),
                stamp,
                self.base_frame,
                self.odom_frame,
            )

        self._last_publish_time = now
        self._last_publish_x = x
        self._last_publish_y = y
        self._last_publish_yaw = yaw


def main():
    rospy.init_node('lepu_chassis_node')
    node = LepuChassisNode()
    rospy.spin()


if __name__ == '__main__':
    main()
