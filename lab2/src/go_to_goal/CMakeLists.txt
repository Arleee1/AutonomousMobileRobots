cmake_minimum_required(VERSION 3.8)
project(go_to_goal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# add nav msgs
find_package(nav_msgs REQUIRED)
# add geometry msgs
find_package(geometry_msgs REQUIRED)
# add sensor msgs
find_package(sensor_msgs REQUIRED)
# add tf2 
find_package(tf2 REQUIRED)



# Add Executables
add_executable(go_to_goal_node src/go_to_goal_node.cpp)

# Add Dependencies
ament_target_dependencies(go_to_goal_node rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2)

# Install Executables
install(TARGETS
  go_to_goal_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
