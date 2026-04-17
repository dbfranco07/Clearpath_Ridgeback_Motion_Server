cmake_minimum_required(VERSION 3.8)
project(ridgeback_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Perception.msg"
  "msg/SafetyStatus.msg"
  "msg/MissionStatus.msg"
  "srv/StoreLocation.srv"
  "srv/QueryLocation.srv"
  "srv/GetAllLocations.srv"
  "srv/MissionCommand.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

# Install Python executables
install(PROGRAMS
  ridgeback_autonomy/safety_controller.py
  ridgeback_autonomy/vlm_perception.py
  ridgeback_autonomy/spatial_memory.py
  ridgeback_autonomy/mission_orchestrator.py
  ridgeback_autonomy/cmd_vel_mux.py
  ridgeback_autonomy/web_dashboard.py
  ridgeback_autonomy/sim_topic_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package for imports
ament_python_install_package(${PROJECT_NAME})

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install worlds
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
