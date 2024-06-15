cmake_minimum_required(VERSION 3.5)
project(lidar_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(restore_from_csv src/restore_from_csv.cpp)
ament_target_dependencies(restore_from_csv rclcpp sensor_msgs std_msgs tf2_ros)

install(TARGETS
  restore_from_csv
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
