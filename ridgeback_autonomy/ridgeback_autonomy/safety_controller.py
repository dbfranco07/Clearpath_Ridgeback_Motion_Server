#!/usr/bin/env python3
"""
Safety Controller Node — Runs on Jetson Orin
==============================================
This is the primary safety layer. It runs independently of all other nodes
and cannot be bypassed. Responsibilities:
  - Monitor LiDAR for obstacles in the robot's path
  - Publish emergency stop velocity overrides when danger is detected
  - Monitor LiDAR heartbeat — stop robot if sensors go silent
  - Publish /safety/status for the web dashboard

Priority: This node starts FIRST and its output feeds the cmd_vel_mux
at the highest priority level.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time

from ridgeback_autonomy.msg import SafetyStatus


class SafetyController(Node):
    def __init__(self):
        super().__init__('safety_controller')

        # Parameters
        self.declare_parameter('danger_zone_m', 0.4)         # Hard stop distance (m)
        self.declare_parameter('warning_zone_m', 0.7)        # Slow-down distance (m)
        self.declare_parameter('lidar_timeout_s', 2.0)       # Max seconds without LiDAR data
        self.declare_parameter('front_cone_deg', 60.0)       # Angular cone to check ahead (±degrees)
        self.declare_parameter('status_rate_hz', 5.0)        # How often to publish status
        self.declare_parameter('lidar_topic', '/r100_0140/sensors/lidar2d_0/scan')
        self.declare_parameter('odom_topic', '/r100_0140/platform/odom/filtered')
        self.declare_parameter('cmd_vel_override_topic', '/safety/cmd_vel_override')
        self.declare_parameter('status_topic', '/safety/status')

        self.danger_zone = self.get_parameter('danger_zone_m').value
        self.warning_zone = self.get_parameter('warning_zone_m').value
        self.lidar_timeout = self.get_parameter('lidar_timeout_s').value
        self.front_cone_rad = math.radians(self.get_parameter('front_cone_deg').value)
        status_rate = self.get_parameter('status_rate_hz').value

        lidar_topic = self.get_parameter('lidar_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        override_topic = self.get_parameter('cmd_vel_override_topic').value
        status_topic = self.get_parameter('status_topic').value

        # QoS
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        reliable_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)

        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, lidar_topic, self._lidar_cb, sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, sensor_qos
        )

        # Publishers
        self.override_pub = self.create_publisher(Twist, override_topic, reliable_qos)
        self.status_pub = self.create_publisher(SafetyStatus, status_topic, reliable_qos)

        # Internal state
        self.closest_obstacle_m = float('inf')
        self.emergency_stop = False
        self.stop_reason = ''
        self.lidar_active = False
        self.last_lidar_time = time.time()
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0

        # Timers
        self.create_timer(1.0 / status_rate, self._status_timer_cb)
        self.create_timer(0.5, self._heartbeat_timer_cb)  # Check LiDAR health at 2 Hz

        self.get_logger().info('Safety Controller started')
        self.get_logger().info(f'  Danger zone: {self.danger_zone}m | Warning zone: {self.warning_zone}m')
        self.get_logger().info(f'  Front cone: ±{self.get_parameter("front_cone_deg").value}°')
        self.get_logger().info(f'  LiDAR timeout: {self.lidar_timeout}s')

    def _lidar_cb(self, msg: LaserScan):
        self.lidar_active = True
        self.last_lidar_time = time.time()

        # Find closest obstacle in the relevant angular cone
        # Consider full surround for side/rear awareness, but prioritize forward cone
        closest_front = float('inf')
        closest_all = float('inf')

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < 0.05:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            # Normalize to [-pi, pi]
            angle = (angle + math.pi) % (2 * math.pi) - math.pi

            closest_all = min(closest_all, r)

            # Forward cone: angle near 0 rad (directly ahead)
            if abs(angle) <= self.front_cone_rad:
                closest_front = min(closest_front, r)

        self.closest_obstacle_m = closest_all

        # Determine safety state
        was_emergency = self.emergency_stop

        if closest_front < self.danger_zone and self.current_linear_vel > 0.0:
            # Obstacle directly ahead while moving forward
            self.emergency_stop = True
            self.stop_reason = f'Obstacle {closest_front:.2f}m ahead (danger zone: {self.danger_zone}m)'
        elif closest_all < self.danger_zone * 0.5:
            # Obstacle extremely close from any direction
            self.emergency_stop = True
            self.stop_reason = f'Obstacle {closest_all:.2f}m (all directions, critical)'
        else:
            self.emergency_stop = False
            self.stop_reason = ''

        if self.emergency_stop:
            self._publish_stop_override()
            if not was_emergency:
                self.get_logger().warn(f'SAFETY STOP: {self.stop_reason}')
        elif was_emergency:
            self.get_logger().info('Safety: obstacle cleared, stop released')

    def _odom_cb(self, msg: Odometry):
        self.current_linear_vel = msg.twist.twist.linear.x
        self.current_angular_vel = msg.twist.twist.angular.z

    def _heartbeat_timer_cb(self):
        """Check if LiDAR is still publishing. If not, trigger emergency stop."""
        elapsed = time.time() - self.last_lidar_time
        if elapsed > self.lidar_timeout and self.lidar_active:
            self.lidar_active = False
            self.emergency_stop = True
            self.stop_reason = f'LiDAR silent for {elapsed:.1f}s (timeout: {self.lidar_timeout}s)'
            self.get_logger().error(f'SAFETY STOP: {self.stop_reason}')
            self._publish_stop_override()

    def _publish_stop_override(self):
        """Publish zero velocity to override any active motion command."""
        msg = Twist()
        # All fields default to 0.0 — explicit zero velocity
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.override_pub.publish(msg)

    def _status_timer_cb(self):
        msg = SafetyStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.is_safe = not self.emergency_stop
        msg.closest_obstacle_m = (
            self.closest_obstacle_m if math.isfinite(self.closest_obstacle_m) else -1.0
        )
        msg.danger_zone_m = self.danger_zone
        msg.lidar_active = self.lidar_active
        msg.emergency_stop_active = self.emergency_stop
        msg.stop_reason = self.stop_reason

        if self.emergency_stop:
            msg.status_text = f'STOP: {self.stop_reason}'
        elif not self.lidar_active:
            msg.status_text = 'WARNING: LiDAR inactive'
        elif self.closest_obstacle_m < self.warning_zone:
            msg.status_text = f'WARNING: obstacle at {self.closest_obstacle_m:.2f}m'
        else:
            msg.status_text = f'OK — closest: {self.closest_obstacle_m:.2f}m'

        self.status_pub.publish(msg)


def main(args=None):
    print('=' * 50)
    print('Ridgeback Autonomy — Safety Controller')
    print('=' * 50)
    rclpy.init(args=args)
    node = SafetyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
