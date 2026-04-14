# Mission orchestrator state
builtin_interfaces/Time stamp
string state              # IDLE, CHECKING_MEMORY, NAVIGATING, EXPLORING, ARRIVED, RETURNING, RETURNED, FAILED, CANCELLED
string target_room        # Room number the robot is trying to reach
string status_text        # Human-readable status for the web UI
float32 progress          # 0.0-1.0 progress toward current goal
float32 goal_x            # Current Nav2 goal position
float32 goal_y
float32 start_x           # Start position this session
float32 start_y
int32 frontiers_explored  # How many frontiers have been explored
int32 rooms_discovered    # Total rooms found so far this session
string[] known_rooms      # Room IDs in memory
