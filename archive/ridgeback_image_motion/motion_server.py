#!/usr/bin/env python3
"""
Motion Service Server - Runs on Ridgeback R100
Receives motion commands via ROS2 service and publishes to /r100_0140/cmd_vel
Supports holonomic (omnidirectional) movement: forward/backward, strafe, rotation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from ridgeback_image_motion.srv import Motion


class MotionServer(Node):
    def __init__(self):
        super().__init__('motion_server')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/r100_0140/cmd_vel')
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # Publisher to cmd_vel (BEST_EFFORT to match twist_mux subscriber)
        cmd_vel_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, cmd_vel_qos)

        # Service server
        self.service = self.create_service(
            Motion,
            'motion_service',
            self.motion_callback
        )

        self.get_logger().info('Motion Service Server started')
        self.get_logger().info(f'  Publishing to: {cmd_vel_topic}')
        self.get_logger().info(f'  Service: motion_service')

    def motion_callback(self, request, response):
        """Handle motion service requests (holonomic)."""
        try:
            twist = Twist()
            twist.linear.x = float(request.linear)
            twist.linear.y = float(request.lateral)
            twist.angular.z = float(request.angular)
            self.cmd_vel_pub.publish(twist)

            if request.linear != 0.0 or request.lateral != 0.0 or request.angular != 0.0:
                self.get_logger().info(
                    f'Motion: linear={request.linear:.3f}, '
                    f'lateral={request.lateral:.3f}, '
                    f'angular={request.angular:.3f}'
                )

            response.success = True
            response.message = 'OK'
        except Exception as e:
            self.get_logger().error(f'Motion error: {e}')
            response.success = False
            response.message = str(e)

        return response


def main(args=None):
    print("=" * 50)
    print("Ridgeback R100 - Motion Service Server")
    print("=" * 50)
    print("Waiting for motion commands...")

    rclpy.init(args=args)
    node = MotionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
