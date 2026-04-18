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
  ridgeback_image_motion/image_publisher.py
  ridgeback_image_motion/web_controller.py
  ridgeback_image_motion/web_dashboard.py
  ridgeback_image_motion/motion_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package for imports
install(DIRECTORY ridgeback_image_motion/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install launch files
install(DIRECTORY ridgeback_image_motion/launch/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)

ament_package()
