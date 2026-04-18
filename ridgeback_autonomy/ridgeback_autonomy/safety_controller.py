#!/usr/bin/env python3
"""
Safety Controller Node — Runs on Jetson Orin
==============================================
This is the primary safety layer. It runs independently of all other nodes
and cannot be bypassed. Responsibilities:
  - Monitor LiDAR for obstacles in the robot's path
  - Publish emergency stop velocity overrides when danger is detected
  - Publish per-axis directional block flags (soft bumper) for the cmd_vel_mux
  - Monitor LiDAR heartbeat — stop robot if sensors go silent
  - Publish /safety/status for the web dashboard

Priority: This node starts FIRST and its output feeds the cmd_vel_mux
at the highest priority level.

Soft bumper design
------------------
LiDAR points are converted to chassis-frame Cartesian (px, py). Four
axis-aligned safety boxes extend 'axis_block_enter_m' beyond the chassis
edge. When any point falls inside a box, the corresponding axis is blocked.
Hysteresis: box shrinks to 'axis_block_exit_m' margin for release, and the
flag stays blocked until 'axis_block_release_scans' consecutive clear scans.

Emergency stop (all-axes) still fires for the critical zone and watchdog.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import time

from ridgeback_autonomy.msg import SafetyStatus


class _AxisFlag:
    """Hysteresis state for one directional block flag."""
    def __init__(self, release_scans: int):
        self.blocked = False
        self._clear_count = 0
        self._release_scans = release_scans

    def update(self, obstacle_present: bool):
        if obstacle_present:
            self.blocked = True
            self._clear_count = 0
        else:
            if self.blocked:
                self._clear_count += 1
                if self._clear_count >= self._release_scans:
                    self.blocked = False
                    self._clear_count = 0
            # already clear → stay clear


class SafetyController(Node):
    def __init__(self):
        super().__init__('safety_controller')

        # ── Legacy parameters ──────────────────────────────────────────────
        self.declare_parameter('danger_zone_m', 0.45)
        self.declare_parameter('warning_zone_m', 0.8)
        self.declare_parameter('lidar_timeout_s', 2.0)
        self.declare_parameter('front_cone_deg', 60.0)
        self.declare_parameter('status_rate_hz', 5.0)
        self.declare_parameter('lidar_topic', '/r100_0140/sensors/lidar2d_0/scan')
        self.declare_parameter('odom_topic', '/r100_0140/platform/odom/filtered')
        self.declare_parameter('cmd_vel_override_topic', '/safety/cmd_vel_override')
        self.declare_parameter('status_topic', '/safety/status')

        # ── Soft-bumper parameters ──────────────────────────────────────────
        self.declare_parameter('chassis_half_x_m', 0.48)
        self.declare_parameter('chassis_half_y_m', 0.40)
        self.declare_parameter('axis_block_enter_m', 0.20)
        self.declare_parameter('axis_block_exit_m', 0.25)
        self.declare_parameter('axis_block_release_scans', 3)
        self.declare_parameter('rotation_corner_margin_m', 0.20)
        self.declare_parameter('rear_blind_default_block', True)
        self.declare_parameter('min_valid_range_m', 0.05)

        self.danger_zone = self.get_parameter('danger_zone_m').value
        self.warning_zone = self.get_parameter('warning_zone_m').value
        self.lidar_timeout = self.get_parameter('lidar_timeout_s').value
        self.front_cone_rad = math.radians(self.get_parameter('front_cone_deg').value)
        status_rate = self.get_parameter('status_rate_hz').value

        lidar_topic = self.get_parameter('lidar_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        override_topic = self.get_parameter('cmd_vel_override_topic').value
        status_topic = self.get_parameter('status_topic').value

        self.hx = self.get_parameter('chassis_half_x_m').value
        self.hy = self.get_parameter('chassis_half_y_m').value
        self.enter_m = self.get_parameter('axis_block_enter_m').value
        self.exit_m = self.get_parameter('axis_block_exit_m').value
        release_scans = self.get_parameter('axis_block_release_scans').value
        self.rot_margin = self.get_parameter('rotation_corner_margin_m').value
        self.rear_default_block = self.get_parameter('rear_blind_default_block').value
        self.min_range = self.get_parameter('min_valid_range_m').value

        # ── QoS ────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        reliable_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        latch_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ── Subscribers ────────────────────────────────────────────────────
        self.lidar_sub = self.create_subscription(
            LaserScan, lidar_topic, self._lidar_cb, sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, sensor_qos
        )
        self.rear_override_sub = self.create_subscription(
            Bool, '/safety/rear_override', self._rear_override_cb, latch_qos
        )
        self.soft_bumper_sub = self.create_subscription(
            Bool, '/safety/soft_bumper_enabled', self._soft_bumper_cb, latch_qos
        )
        self.safety_enabled_sub = self.create_subscription(
            Bool, '/safety/enabled', self._safety_enabled_cb, latch_qos
        )

        # ── Publishers ─────────────────────────────────────────────────────
        self.override_pub = self.create_publisher(Twist, override_topic, reliable_qos)
        self.status_pub = self.create_publisher(SafetyStatus, status_topic, reliable_qos)

        # ── Internal state ──────────────────────────────────────────────────
        self.closest_obstacle_m = float('inf')
        self.emergency_stop = False
        self.stop_reason = ''
        self.lidar_active = False
        self.last_lidar_time = time.time()
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.rear_override = False
        self.soft_bumper_enabled = True
        self.safety_enabled = True

        # Per-axis hysteresis flags
        self._ax = {
            'pos_x': _AxisFlag(release_scans),
            'neg_x': _AxisFlag(release_scans),
            'pos_y': _AxisFlag(release_scans),
            'neg_y': _AxisFlag(release_scans),
            'pos_yaw': _AxisFlag(release_scans),
            'neg_yaw': _AxisFlag(release_scans),
        }

        # Clearance values published in status (body-edge distance, m)
        self._clr = {'fwd': -1.0, 'rev': -1.0, 'left': -1.0, 'right': -1.0}

        # ── Timers ─────────────────────────────────────────────────────────
        self.create_timer(1.0 / status_rate, self._status_timer_cb)
        self.create_timer(0.5, self._heartbeat_timer_cb)

        self.get_logger().info('Safety Controller started')
        self.get_logger().info(
            f'  Emergency stop zone: {self.danger_zone}m | Warning: {self.warning_zone}m'
        )
        self.get_logger().info(
            f'  Soft bumper: block at {self.enter_m}m, release at {self.exit_m}m '
            f'(chassis {self.hx}m×{self.hy}m)'
        )
        self.get_logger().info(
            f'  Rear blind policy: {"BLOCK" if self.rear_default_block else "ALLOW"}'
        )

    # ── Subscriber callbacks ────────────────────────────────────────────────

    def _rear_override_cb(self, msg: Bool):
        prev = self.rear_override
        self.rear_override = msg.data
        if self.rear_override != prev:
            self.get_logger().info(
                f'Rear override: {"ENABLED — reverse allowed" if self.rear_override else "DISABLED — reverse blocked"}'
            )

    def _soft_bumper_cb(self, msg: Bool):
        prev = self.soft_bumper_enabled
        self.soft_bumper_enabled = msg.data
        if self.soft_bumper_enabled != prev:
            self.get_logger().info(
                f'Soft bumper: {"ENABLED — directional blocking active" if self.soft_bumper_enabled else "DISABLED — directional blocking suppressed"}'
            )

    def _safety_enabled_cb(self, msg: Bool):
        prev = self.safety_enabled
        self.safety_enabled = msg.data
        if self.safety_enabled != prev:
            if self.safety_enabled:
                self.get_logger().warn('Safety ENABLED — all enforcement active')
            else:
                self.get_logger().warn('Safety DISABLED — testing mode, no emergency stops')
                # Clear any active e-stop since enforcement is off
                self.emergency_stop = False
                self.stop_reason = ''

    def _odom_cb(self, msg: Odometry):
        self.current_linear_vel = msg.twist.twist.linear.x
        self.current_angular_vel = msg.twist.twist.angular.z

    def _lidar_cb(self, msg: LaserScan):
        self.lidar_active = True
        self.last_lidar_time = time.time()

        hx, hy = self.hx, self.hy
        enter = self.enter_m
        exit_ = self.exit_m
        min_r = self.min_range

        # Per-axis obstacle detection accumulators.
        # Each flag has an "enter" zone (assert block) and "exit" zone (allow release).
        # A point that is inside the enter box also counts as "not clear for exit".
        obs_enter = {k: False for k in self._ax}
        obs_occupied = {k: False for k in self._ax}  # inside enter OR between enter/exit

        # Running min LiDAR distance to body edge per direction (for clearance display)
        min_fwd = float('inf')    # min(px - hx) for points with px > hx
        min_rev = float('inf')    # min(-px - hx) for points with px < -hx
        min_left = float('inf')   # min(py - hy) for points with py > hy
        min_right = float('inf')  # min(-py - hy) for points with py < -hy

        closest_all = float('inf')

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < min_r:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            angle = (angle + math.pi) % (2 * math.pi) - math.pi

            px = r * math.cos(angle)
            py = r * math.sin(angle)

            closest_all = min(closest_all, r)

            # ── Forward (+x) ───────────────────────────────────────────────
            # Safety box: px ∈ [hx, hx+enter], |py| ≤ hy
            if hx < px and abs(py) <= hy:
                dist_to_edge = px - hx
                min_fwd = min(min_fwd, dist_to_edge)
                if dist_to_edge <= enter:
                    obs_enter['pos_x'] = True
                if dist_to_edge <= exit_:
                    obs_occupied['pos_x'] = True

            # ── Left (+y) ──────────────────────────────────────────────────
            if hy < py and abs(px) <= hx:
                dist_to_edge = py - hy
                min_left = min(min_left, dist_to_edge)
                if dist_to_edge <= enter:
                    obs_enter['pos_y'] = True
                if dist_to_edge <= exit_:
                    obs_occupied['pos_y'] = True

            # ── Right (-y) ─────────────────────────────────────────────────
            if py < -hy and abs(px) <= hx:
                dist_to_edge = -py - hy
                min_right = min(min_right, dist_to_edge)
                if dist_to_edge <= enter:
                    obs_enter['neg_y'] = True
                if dist_to_edge <= exit_:
                    obs_occupied['neg_y'] = True

            # ── Reverse (-x) ───────────────────────────────────────────────
            if px < -hx and abs(py) <= hy:
                dist_to_edge = -px - hx
                min_rev = min(min_rev, dist_to_edge)
                if dist_to_edge <= enter:
                    obs_enter['neg_x'] = True
                if dist_to_edge <= exit_:
                    obs_occupied['neg_x'] = True

            # ── Rotation — CCW (+yaw): leading corners FL (+hx,+hy) and RR (-hx,-hy) ──
            rot_m = self.rot_margin
            d_fl = math.hypot(px - hx, py - hy)
            d_rr = math.hypot(px + hx, py + hy)
            if d_fl <= enter + rot_m:
                obs_enter['pos_yaw'] = True
            if d_fl <= exit_ + rot_m:
                obs_occupied['pos_yaw'] = True
            if d_rr <= enter + rot_m and not self.rear_default_block:
                obs_enter['pos_yaw'] = True
            if d_rr <= exit_ + rot_m and not self.rear_default_block:
                obs_occupied['pos_yaw'] = True

            # ── Rotation — CW (-yaw): leading corners FR (+hx,-hy) and RL (-hx,+hy) ──
            d_fr = math.hypot(px - hx, py + hy)
            d_rl = math.hypot(px + hx, py - hy)
            if d_fr <= enter + rot_m:
                obs_enter['neg_yaw'] = True
            if d_fr <= exit_ + rot_m:
                obs_occupied['neg_yaw'] = True
            if d_rl <= enter + rot_m and not self.rear_default_block:
                obs_enter['neg_yaw'] = True
            if d_rl <= exit_ + rot_m and not self.rear_default_block:
                obs_occupied['neg_yaw'] = True

        self.closest_obstacle_m = closest_all

        # Update hysteresis flags.
        # For release: flag stays blocked while obs_occupied is True.
        # For assert:  flag sets blocked immediately when obs_enter is True.
        if self.soft_bumper_enabled:
            for key, flag in self._ax.items():
                # Pass obs_enter as "obstacle_present" — drives immediate assert.
                # Also force "still occupied" when in the hysteresis exit band.
                flag.update(obs_enter[key] or (flag.blocked and obs_occupied[key]))

            # Rear blind-spot default: -x is blocked unless operator enables rear_override.
            # Rotation's rear-corner (RR/RL) contribution is already excluded from the scan loop
            # when rear_default_block=True, so only the front corners gate rotation here.
            if self.rear_default_block and not self.rear_override:
                self._ax['neg_x'].blocked = True
        else:
            # Soft bumper disabled: clear all axis flags immediately.
            # Emergency e-stop (closest_all / front cone) remains fully active.
            for flag in self._ax.values():
                flag.blocked = False
                flag._clear_count = 0

        # Store clearances (body-edge distance; -1 = blind / no measurement)
        def _clr_val(v):
            return round(v, 3) if math.isfinite(v) else -1.0
        self._clr['fwd'] = _clr_val(min_fwd)
        self._clr['rev'] = _clr_val(min_rev)
        self._clr['left'] = _clr_val(min_left)
        self._clr['right'] = _clr_val(min_right)

        # ── Legacy emergency stop ─────────────────────────────────────────
        was_emergency = self.emergency_stop

        if not self.safety_enabled:
            # Safety enforcement disabled — never trigger e-stop, keep telemetry only.
            self.emergency_stop = False
            self.stop_reason = ''
        elif closest_all < self.danger_zone * 0.5:
            self.emergency_stop = True
            self.stop_reason = f'Obstacle {closest_all:.2f}m — critical zone'
        elif (self._is_front_obstacle_within(msg, self.danger_zone)
              and self.current_linear_vel > 0.0):
            self.emergency_stop = True
            self.stop_reason = f'Obstacle ahead in danger zone ({self.danger_zone}m)'
        else:
            self.emergency_stop = False
            self.stop_reason = ''

        if self.emergency_stop:
            self._publish_stop_override()
            if not was_emergency:
                self.get_logger().warn(f'SAFETY STOP: {self.stop_reason}')
        elif was_emergency:
            self.get_logger().info('Safety: obstacle cleared, stop released')

    def _is_front_obstacle_within(self, msg: LaserScan, dist: float) -> bool:
        """Return True if any range within the front cone is closer than dist."""
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < self.min_range:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            angle = (angle + math.pi) % (2 * math.pi) - math.pi
            if abs(angle) <= self.front_cone_rad and r < dist:
                return True
        return False

    # ── Heartbeat ───────────────────────────────────────────────────────────

    def _heartbeat_timer_cb(self):
        elapsed = time.time() - self.last_lidar_time
        if elapsed > self.lidar_timeout and self.lidar_active:
            self.lidar_active = False
            if self.safety_enabled:
                self.emergency_stop = True
                self.stop_reason = f'LiDAR silent for {elapsed:.1f}s (timeout: {self.lidar_timeout}s)'
                self.get_logger().error(f'SAFETY STOP: {self.stop_reason}')
                self._publish_stop_override()
            else:
                self.get_logger().warn(
                    f'LiDAR silent for {elapsed:.1f}s — safety disabled, no e-stop'
                )

    # ── Publishing ──────────────────────────────────────────────────────────

    def _publish_stop_override(self):
        msg = Twist()
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

        msg.safety_enabled = self.safety_enabled

        if not self.safety_enabled:
            msg.status_text = 'SAFETY DISABLED — testing mode'
        elif self.emergency_stop:
            msg.status_text = f'STOP: {self.stop_reason}'
        elif not self.lidar_active:
            msg.status_text = 'WARNING: LiDAR inactive'
        elif self.closest_obstacle_m < self.warning_zone:
            msg.status_text = f'WARNING: obstacle at {self.closest_obstacle_m:.2f}m'
        else:
            msg.status_text = f'OK — closest: {self.closest_obstacle_m:.2f}m'

        # Per-axis directional block flags
        msg.block_pos_x = self._ax['pos_x'].blocked
        msg.block_neg_x = self._ax['neg_x'].blocked
        msg.block_pos_y = self._ax['pos_y'].blocked
        msg.block_neg_y = self._ax['neg_y'].blocked
        msg.block_pos_yaw = self._ax['pos_yaw'].blocked
        msg.block_neg_yaw = self._ax['neg_yaw'].blocked

        msg.clearance_forward_m = self._clr['fwd']
        msg.clearance_reverse_m = self._clr['rev']
        msg.clearance_left_m = self._clr['left']
        msg.clearance_right_m = self._clr['right']
        msg.soft_bumper_enabled = self.soft_bumper_enabled

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
