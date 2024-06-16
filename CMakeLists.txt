cmake_minimum_required(VERSION 3.0.2)
project(lidar_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_executable(restore_from_csv src/restore_from_csv.cpp)
# target_link_libraries(restore_from_csv ${catkin_LIBRARIES})

# install(TARGETS restore_from_csv
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
