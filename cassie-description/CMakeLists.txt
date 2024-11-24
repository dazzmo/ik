cmake_minimum_required(VERSION 3.8)

project(cassie_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY launch urdf srdf meshes rviz
        DESTINATION share/${PROJECT_NAME}
)

ament_package()