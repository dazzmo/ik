cmake_minimum_required(VERSION 3.8)
project(ik_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(pinocchio REQUIRED)


find_package(ik REQUIRED)

include_directories(include)

# Cassie Example
add_executable(cassie_ik 
src/cassie.cpp
src/rviz_model_loader.cpp
)
ament_target_dependencies(cassie_ik
  geometry_msgs
  sensor_msgs
  rclcpp
  rclpy
  std_msgs
  tf2_ros
  visualization_msgs
  robot_state_publisher
  pinocchio
)
target_link_libraries(cassie_ik ik::ik)
install(TARGETS cassie_ik
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
