cmake_minimum_required(VERSION 3.8)
project(ridgeback_image_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Motion.srv"
)

# Install Python executables
install(PROGRAMS
  ridgeback_image_motion/autonomy_common.py
  ridgeback_image_motion/cmd_vel_mux.py
  ridgeback_image_motion/frontier_explorer.py
  ridgeback_image_motion/image_publisher.py
  ridgeback_image_motion/mission_orchestrator.py
  ridgeback_image_motion/room_detector.py
  ridgeback_image_motion/safety_controller.py
  ridgeback_image_motion/web_controller.py
  ridgeback_image_motion/web_dashboard.py
  ridgeback_image_motion/motion_server.py
  ridgeback_image_motion/spatial_memory.py
  ridgeback_image_motion/vlm_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY ridgeback_image_motion/launch/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

ament_package()
