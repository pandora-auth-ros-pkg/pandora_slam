cmake_minimum_required(VERSION 2.8.3)
project(pandora_slam_3d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf
  laser_geometry
  octomap_server
  cv_bridge
  dynamic_reconfigure
  roslint
  pandora_vision_hole_detector
)

set(CMAKE_BUILD_TYPE Release)
#~ set(CMAKE_BUILD_TYPE Debug)

generate_dynamic_reconfigure_options(
  cfg/point_cloud_subsampler.cfg)

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

add_executable(
  point_cloud_aggregator
  src/point_cloud_aggregator/point_cloud_aggregator.cpp)

target_link_libraries(
  point_cloud_aggregator
  ${catkin_LIBRARIES})

add_executable(
  point_cloud_subsampler
  src/point_cloud_subsampler/point_cloud_subsampler.cpp)

add_dependencies(
  point_cloud_subsampler
  ${PROJECT_NAME}_gencfg)

target_link_libraries(
  point_cloud_subsampler
  utils_lib
  ${catkin_LIBRARIES})

add_library(
  randomized_transform
  src/matching_octomap_server/randomized_transform.cpp)

target_link_libraries(
  randomized_transform
  ${catkin_LIBRARIES})

add_executable(
  matching_octomap_server
  src/matching_octomap_server/matching_octomap_server.cpp)

target_link_libraries(
  matching_octomap_server
  randomized_transform
  utils_lib
  ${catkin_LIBRARIES})

# To check lint : catkin_make roslint_pandora_slam_3d
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use, -whitespace/comments, -runtime/references, -runtime/arrays, -whitespace/empty_loop_body")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  *.h
  *.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
