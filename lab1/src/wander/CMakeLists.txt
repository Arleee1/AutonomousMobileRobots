cmake_minimum_required(VERSION 3.8)
project(wander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)

# Add executables
add_executable(wander_node src/wander_node.cpp)
add_executable(ir_distance_node src/ir_distance_node.cpp)

# Specify libraries to link against
set(dependencies
  rclcpp
  sensor_msgs
)

# For wander_node
ament_target_dependencies(wander_node
  ${dependencies}
  geometry_msgs
)

# For ir_distance_node
ament_target_dependencies(ir_distance_node
  ${dependencies}
  irobot_create_msgs
)

# Install the executables
install(TARGETS
  wander_node
  ir_distance_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_lint_auto_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
