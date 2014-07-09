cmake_minimum_required(VERSION 2.8.3)
project(pandora_slam_3d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  laser_geometry
  octomap_server
)

set(CMAKE_BUILD_TYPE Debug)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pandora_3d_slam
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  laser_scan_to_point_cloud_converter
  src/laser_scan_to_point_cloud_converter/laser_scan_to_point_cloud_converter.cpp)

target_link_libraries(
  laser_scan_to_point_cloud_converter
  ${catkin_LIBRARIES})
