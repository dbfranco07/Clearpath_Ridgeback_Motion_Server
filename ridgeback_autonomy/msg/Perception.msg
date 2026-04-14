# VLM Perception output message
builtin_interfaces/Time stamp
string description          # Full VLM response text
string[] detected_rooms     # Room numbers detected (e.g. ["305", "306"])
float32[] room_confidence   # Confidence per detected room (0.0-1.0)
string environment_type     # "hallway", "intersection", "room", "open_area", "unknown"
bool has_obstacles          # True if VLM reports obstacles (people, chairs, etc.)
string image_id             # Frame identifier for tracing back to source image
float32 robot_x             # Robot pose at time of capture (map frame)
float32 robot_y
float32 robot_yaw
