# Clearpath Ridgeback R100 — ROS2 Humble Project Context

## Robot Identity
- **Robot Model:** Clearpath Ridgeback R100
- **Serial Number:** r100-0140
- **Hostname:** cpr-r100-0140
- **OS:** Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- **ROS Distribution:** ROS2 Humble
- **ROS Namespace:** `r100_0140`
- **MCU Firmware:** 1.3.1

---

## Network & Access
| Connection | IP Address | Notes |
|---|---|---|
| Internal Bridge (Ethernet) | `192.168.131.1` | Primary — always available |
| WiFi | `10.158.39.184` | May change — check `ip a` |
| Front LiDAR | `192.168.131.20` | Ethernet connected |
| MCU | `192.168.131.2` | Internal only |

```bash
# SSH into robot
ssh administrator@192.168.131.1
# Password: clearpath

# Always source ROS2 after login
source /opt/ros/humble/setup.bash
```

---

## Hardware Inventory
| Component | Model | Connection | Notes |
|---|---|---|---|
| Platform | Ridgeback R100 | Internal | Omnidirectional (holonomic) |
| Front LiDAR | Hokuyo UST-10LX | Ethernet `192.168.131.20` | 270° scan, 25Hz |
| RGB-D Camera | Intel RealSense D435 | USB | Serial: 317222071726 |
| IMU | Built-in | Internal | 50Hz |
| Controller | PS4 Bluetooth | Bluetooth | |
| CPU | Intel Core i3-9100TE @ 2.20GHz | — | 4 cores |
| Drive | WD Blue SA510 M.2 250GB | — | |

---

## Key ROS2 Topics

### Drive & Control
```
/r100_0140/cmd_vel                          geometry_msgs/msg/Twist       # Main drive input
/r100_0140/platform/cmd_vel_unstamped       geometry_msgs/msg/Twist       # Unstamped alternative
/r100_0140/platform/odom                    nav_msgs/msg/Odometry         # Raw odometry
/r100_0140/platform/odom/filtered           nav_msgs/msg/Odometry         # EKF-filtered odometry
/r100_0140/platform/emergency_stop          std_msgs/msg/Bool             # E-Stop state
```

### Platform Status
```
/r100_0140/platform/mcu/status              clearpath_platform_msgs/msg/Status      # MCU status
/r100_0140/platform/mcu/status/stop         clearpath_platform_msgs/msg/StopStatus  # Stop loop
/r100_0140/platform/mcu/status/power        clearpath_platform_msgs/msg/Power       # Power/battery
/r100_0140/platform/bms/state               sensor_msgs/msg/BatteryState            # Battery state
/r100_0140/platform/puma/status             clearpath_motor_msgs/msg/PumaMultiStatus # Motor status
/r100_0140/platform/puma/feedback           clearpath_motor_msgs/msg/PumaMultiFeedback
```

### Sensors
```
/r100_0140/sensors/lidar2d_0/scan           sensor_msgs/msg/LaserScan     # Front LiDAR (25Hz)
/r100_0140/sensors/lidar2d_0/diagnostics    diagnostic_msgs/msg/DiagnosticArray
/r100_0140/sensors/imu_0/data               sensor_msgs/msg/Imu           # IMU (50Hz)
/r100_0140/sensors/imu_0/data_raw           sensor_msgs/msg/Imu           # Raw IMU
/r100_0140/sensors/imu_0/magnetic_field     sensor_msgs/msg/MagneticField
/r100_0140/sensors/camera_0/color/image     sensor_msgs/msg/Image         # RGB (30Hz)
/r100_0140/sensors/camera_0/depth/image     sensor_msgs/msg/Image         # Depth (15Hz)
/r100_0140/sensors/camera_0/points          sensor_msgs/msg/PointCloud2   # Pointcloud (15Hz)
```

### Transforms & State
```
/r100_0140/tf                               tf2_msgs/msg/TFMessage
/r100_0140/tf_static                        tf2_msgs/msg/TFMessage
/r100_0140/robot_description                std_msgs/msg/String
/r100_0140/joint_states                     sensor_msgs/msg/JointState
```

---

## Key ROS2 Services
```
/r100_0140/platform/mcu/configure           clearpath_platform_msgs/srv/ConfigureMcu
/r100_0140/enable                           std_srvs/srv/Trigger
/r100_0140/toggle                           std_srvs/srv/Trigger
/r100_0140/set_pose                         clearpath_platform_msgs/srv/SetPose
```

---

## URDF & Robot Description
- **URDF Xacro:** `/etc/clearpath/robot.urdf.xacro`
- **SRDF:** `/etc/clearpath/robot.srdf`
- **robot.yaml:** `/etc/clearpath/robot.yaml`

### Available URDF Links (parent frames for sensors/attachments)
```
base_link, chassis_link, top_link, riser_link,
front_mount (via accessories), rear_mount (via accessories),
default_mount, axle_link
```

### Current robot.yaml Configuration
```yaml
serial_number: r100-0140
version: 0
system:
  hosts:
    - hostname: cpr-r100-0140
      ip: 192.168.131.1
  ros2:
    namespace: r100_0140
platform:
  controller: ps4
sensors:
  lidar2d:
    - model: hokuyo_ust
      urdf_enabled: true
      launch_enabled: true
      parent: chassis_link
      xyz: [0.0, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]
  camera:
    - model: intel_realsense
      urdf_enabled: true
      launch_enabled: true
      parent: top_link
      xyz: [0.43, 0.05, 0.01]
      rpy: [0.0, 0.0, 0.0]
```

---

## Custom Package
- **Package:** `jhic02_ridgeback`
- **GitLab:** `https://gitlab.clearpathrobotics.com/ridgeback_customization/jhic02_ridgeback.git`
- **Version:** 0.0.1
- Contains custom camera mount URDF and mesh files

---

## Systemd Services
```bash
# Main service (starts everything)
sudo systemctl start/stop/restart clearpath-robot.service

# Platform nodes (drives, IMU, EKF, etc.)
sudo systemctl status clearpath-platform

# Sensor nodes (LiDAR, camera)
sudo systemctl status clearpath-sensors

# View live logs
journalctl -u clearpath-platform -f
journalctl -u clearpath-sensors -f
```

---

## Driving the Robot

### Keyboard Teleop
```bash
sudo apt-get install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/r100_0140/cmd_vel
```

### Publish Drive Command
```bash
# Forward at 0.2 m/s
ros2 topic pub /r100_0140/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10

# Rotate at 0.5 rad/s
ros2 topic pub /r100_0140/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}' -r 10

# Strafe left (holonomic)
ros2 topic pub /r100_0140/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10

# Stop
ros2 topic pub /r100_0140/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --once
```

---

## Python Node Templates

### Publisher (Drive Command)
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RidgebackPublisher(Node):
    def __init__(self):
        super().__init__('ridgeback_publisher')
        self.publisher = self.create_publisher(
            Twist, '/r100_0140/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_cmd)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = 0.2   # m/s forward
        msg.linear.y = 0.0   # m/s strafe (holonomic)
        msg.angular.z = 0.0  # rad/s rotation
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = RidgebackPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber (LiDAR)
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/r100_0140/sensors/lidar2d_0/scan',
            self.lidar_callback, 10)

    def lidar_callback(self, msg):
        ranges = [r for r in msg.ranges if r > 0.1 and r < float('inf')]
        if ranges:
            self.get_logger().info(f'Closest: {min(ranges):.2f}m')

def main():
    rclpy.init()
    node = LidarSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber (Camera Image)
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/r100_0140/sensors/camera_0/color/image',
            self.image_callback, 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('RealSense', cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber (IMU)
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/r100_0140/sensors/imu_0/data',
            self.imu_callback, 10)

    def imu_callback(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        self.get_logger().info(f'Accel: x={ax:.2f} y={ay:.2f} z={az:.2f}')

def main():
    rclpy.init()
    node = ImuSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Troubleshooting Quick Reference

### Robot won't move
```bash
# 1. Check stop loop
ros2 topic echo /r100_0140/platform/mcu/status/stop --once
# If stop_power_status: true → release E-Stop + press Stop Reset button

# 2. Check controllers
ros2 topic list | grep velocity

# 3. Restart services
sudo systemctl restart clearpath-robot.service
```

### Sensor not publishing
```bash
# Check sensors service
sudo systemctl status clearpath-sensors

# Check LiDAR reachable
ping 192.168.131.20 -c 3

# Restart
sudo systemctl restart clearpath-robot.service
```

### Robot light colors (ROS2)
- **Front WHITE + Rear RED** = Normal operating state ✅
- **PURPLE/BLUE** = Normal ROS2 state ✅
- **All RED solid** = E-Stop engaged ❌
- **RED blinking** = ROS not running ❌
- **YELLOW** = Charging 🔋
