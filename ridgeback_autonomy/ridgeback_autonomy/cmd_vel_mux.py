#!/usr/bin/env python3
"""
Command Velocity Multiplexer — Runs on Jetson Orin
====================================================
Arbitrates between multiple velocity sources with priority:
  1. Safety controller override (HIGHEST — emergency stop)
  2. Nav2 autonomous navigation
  3. Web teleop (manual control via web dashboard)

Publishes the winning velocity directly to /r100_0140/cmd_vel over DDS.
This approach keeps all arbitration on the Jetson side before commands
reach the Ridgeback.

Timeout-based locking: if a higher-priority source publishes, it locks
out lower-priority sources for a configurable duration.

After source selection, per-axis directional block flags from
/safety/status are applied to the winning Twist. This allows the robot to
move freely on unblocked axes even when one axis is near an obstacle.
Emergency stop (safety_active) still zeroes all axes.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from ridgeback_autonomy.msg import SafetyStatus
import time


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        # Parameters
        self.declare_parameter('output_topic', '/r100_0140/cmd_vel')
        self.declare_parameter('safety_timeout_s', 1.0)
        self.declare_parameter('nav2_timeout_s', 0.5)
        self.declare_parameter('teleop_timeout_s', 0.3)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('apply_axis_mask', True)
        self.declare_parameter('axis_mask_status_timeout_s', 0.5)

        output_topic = self.get_parameter('output_topic').value
        self.safety_timeout = self.get_parameter('safety_timeout_s').value
        self.nav2_timeout = self.get_parameter('nav2_timeout_s').value
        self.teleop_timeout = self.get_parameter('teleop_timeout_s').value
        publish_rate = self.get_parameter('publish_rate_hz').value
        self.apply_axis_mask = self.get_parameter('apply_axis_mask').value
        self.mask_timeout = self.get_parameter('axis_mask_status_timeout_s').value

        # QoS
        reliable_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        best_effort_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.safety_sub = self.create_subscription(
            Twist, '/safety/cmd_vel_override', self._safety_cb, reliable_qos
        )
        self.safety_status_sub = self.create_subscription(
            SafetyStatus, '/safety/status', self._safety_status_cb, reliable_qos
        )
        self.nav2_sub = self.create_subscription(
            Twist, '/cmd_vel_nav2', self._nav2_cb, best_effort_qos
        )
        self.teleop_sub = self.create_subscription(
            Twist, '/cmd_vel_teleop', self._teleop_cb, best_effort_qos
        )

        # Publisher to Ridgeback
        output_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.output_pub = self.create_publisher(Twist, output_topic, output_qos)

        # State
        self.safety_active = False
        self.safety_cmd = Twist()
        self.nav2_cmd = Twist()
        self.teleop_cmd = Twist()

        self.safety_last_time = 0.0
        self.nav2_last_time = 0.0
        self.teleop_last_time = 0.0
        self.status_last_time = 0.0

        self.active_source = 'none'

        # Per-axis block flags (from /safety/status)
        self._block_pos_x = False
        self._block_neg_x = False
        self._block_pos_y = False
        self._block_neg_y = False
        self._block_pos_yaw = False
        self._block_neg_yaw = False

        # Publish timer
        self.create_timer(1.0 / publish_rate, self._publish_timer_cb)

        self.get_logger().info('Cmd Vel Mux started')
        self.get_logger().info(f'  Output: {output_topic} @ {publish_rate}Hz')
        self.get_logger().info('  Priority: safety > nav2 > teleop')
        self.get_logger().info(
            f'  Per-axis mask: {"enabled" if self.apply_axis_mask else "disabled"}'
        )

    def _safety_cb(self, msg: Twist):
        self.safety_cmd = msg
        self.safety_last_time = time.time()

    def _safety_status_cb(self, msg: SafetyStatus):
        self.safety_active = msg.emergency_stop_active
        self.status_last_time = time.time()
        self._block_pos_x = msg.block_pos_x
        self._block_neg_x = msg.block_neg_x
        self._block_pos_y = msg.block_pos_y
        self._block_neg_y = msg.block_neg_y
        self._block_pos_yaw = msg.block_pos_yaw
        self._block_neg_yaw = msg.block_neg_yaw

    def _nav2_cb(self, msg: Twist):
        if not self.safety_active:
            self.nav2_cmd = msg
            self.nav2_last_time = time.time()

    def _teleop_cb(self, msg: Twist):
        if not self.safety_active:
            self.teleop_cmd = msg
            self.teleop_last_time = time.time()

    def _apply_axis_mask(self, twist: Twist) -> Twist:
        """Zero out only the commanded components whose axis is blocked.

        Safety-status stale failsafe: if the status message is older than
        mask_timeout, treat all axes as blocked to avoid commanding into
        an unknown obstacle state.
        """
        out = Twist()
        out.linear.x = twist.linear.x
        out.linear.y = twist.linear.y
        out.linear.z = twist.linear.z
        out.angular.x = twist.angular.x
        out.angular.y = twist.angular.y
        out.angular.z = twist.angular.z

        status_stale = (time.time() - self.status_last_time) > self.mask_timeout

        if status_stale:
            # No recent safety data — fail safe: zero everything
            out.linear.x = 0.0
            out.linear.y = 0.0
            out.angular.z = 0.0
            return out

        if out.linear.x > 0 and self._block_pos_x:
            out.linear.x = 0.0
        if out.linear.x < 0 and self._block_neg_x:
            out.linear.x = 0.0
        if out.linear.y > 0 and self._block_pos_y:
            out.linear.y = 0.0
        if out.linear.y < 0 and self._block_neg_y:
            out.linear.y = 0.0
        if out.angular.z > 0 and self._block_pos_yaw:
            out.angular.z = 0.0
        if out.angular.z < 0 and self._block_neg_yaw:
            out.angular.z = 0.0

        return out

    def _publish_timer_cb(self):
        now = time.time()
        output = Twist()

        # Priority 1: Safety override (full stop)
        if self.safety_active or (now - self.safety_last_time < self.safety_timeout):
            output = self.safety_cmd  # Always zero Twist from safety controller
            new_source = 'safety'

        # Priority 2: Nav2
        elif now - self.nav2_last_time < self.nav2_timeout:
            output = self.nav2_cmd
            if self.apply_axis_mask:
                output = self._apply_axis_mask(output)
            new_source = 'nav2'

        # Priority 3: Teleop
        elif now - self.teleop_last_time < self.teleop_timeout:
            output = self.teleop_cmd
            if self.apply_axis_mask:
                output = self._apply_axis_mask(output)
            new_source = 'teleop'

        else:
            output = Twist()
            new_source = 'none'

        if new_source != self.active_source:
            self.get_logger().info(f'Mux: source switched {self.active_source} → {new_source}')
            self.active_source = new_source

        self.output_pub.publish(output)


def main(args=None):
    print('=' * 50)
    print('Ridgeback Autonomy — Cmd Vel Mux')
    print('=' * 50)
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
