cmake_minimum_required(VERSION 2.8.3)
project(pandora_pose_estimation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  roslint
  state_manager
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS
    roscpp
    tf
    sensor_msgs
    roslint
    state_manager
    std_msgs
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

################################################################################
#                               Pose Estimation                                #
################################################################################

add_library(pose_estimation src/pose_estimation.cpp)
target_link_libraries(pose_estimation
  ${CATKIN_LIBRARIES}
  )

add_executable(pose_estimation_node src/pose_estimation_node.cpp)
target_link_libraries(pose_estimation_node
  ${catkin_LIBRARIES}
  pose_estimation
  )

# set(ROSLINT_CPP_OPTS
    # "--filter=-whitespace/end_of_line,-build/include_order,-whitespace/blank_line,-whitespace/parens,-whitespace/comments,-whitespace/braces")
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
    src/*.cpp
  )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
