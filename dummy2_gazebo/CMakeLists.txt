cmake_minimum_required(VERSION 3.8)
project(dummy2_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch urdf worlds config bt src
  DESTINATION share/${PROJECT_NAME}
)

add_executable(exploration_node src/exploration_node.cpp)
ament_target_dependencies(exploration_node rclcpp nav2_costmap_2d geometry_msgs)

add_executable(test_node src/test.cpp)
ament_target_dependencies(test_node rclcpp nav_msgs nav2_msgs geometry_msgs visualization_msgs rclcpp_action)

add_executable(test2_node src/test2.cpp)
ament_target_dependencies(test2_node rclcpp nav_msgs nav2_msgs geometry_msgs visualization_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(initial_pose_publisher src/initialpose_node.cpp)
ament_target_dependencies(initial_pose_publisher rclcpp geometry_msgs tf2 tf2_geometry_msgs)

install(TARGETS
  exploration_node
  test_node
  initial_pose_publisher
  test2_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
