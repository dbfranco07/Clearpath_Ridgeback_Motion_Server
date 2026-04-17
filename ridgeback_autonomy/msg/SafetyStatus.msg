# Safety controller status
builtin_interfaces/Time stamp
bool is_safe                   # True = robot may move, False = emergency stop active
float32 closest_obstacle_m     # Closest obstacle distance in meters
float32 danger_zone_m          # Configured danger zone radius
string status_text             # Human-readable status description
bool lidar_active              # True if LiDAR data is being received
bool emergency_stop_active     # True if emergency stop is currently engaged
string stop_reason             # Why the stop was triggered (if applicable)

# Per-axis directional block flags (soft bumper, 0.2 m from robot body edge)
# True = movement in this direction is forbidden; other axes unaffected
bool block_pos_x               # Forward  (+x) blocked
bool block_neg_x               # Reverse  (-x) blocked (includes rear-blind default)
bool block_pos_y               # Strafe left  (+y) blocked
bool block_neg_y               # Strafe right (-y) blocked
bool block_pos_yaw             # CCW rotation (+yaw) blocked
bool block_neg_yaw             # CW  rotation (-yaw) blocked

# Clearance to robot body edge in each axis direction (m); -1.0 = unknown / blind
float32 clearance_forward_m
float32 clearance_reverse_m
float32 clearance_left_m
float32 clearance_right_m
