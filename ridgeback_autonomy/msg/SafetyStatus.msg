# Safety controller status
builtin_interfaces/Time stamp
bool is_safe                   # True = robot may move, False = emergency stop active
float32 closest_obstacle_m     # Closest obstacle distance in meters
float32 danger_zone_m          # Configured danger zone radius
string status_text             # Human-readable status description
bool lidar_active              # True if LiDAR data is being received
bool emergency_stop_active     # True if emergency stop is currently engaged
string stop_reason             # Why the stop was triggered (if applicable)
