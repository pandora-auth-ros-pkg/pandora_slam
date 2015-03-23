# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pandora_slam_2d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  state_manager
  diagnostic_updater
  state_manager_msgs
  crsm_slam
  roslint
  )


catkin_package(
  CATKIN_DEPENDS
    roscpp
    tf
    state_manager
    diagnostic_updater
    state_manager_msgs
    crsm_slam
  INCLUDE_DIRS
    include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

############ pandora_slam_2d ##############

add_library(${PROJECT_NAME}_pandora_slam_2d
  src/pandora_slam_2d.cpp)
target_link_libraries(${PROJECT_NAME}_pandora_slam_2d
  ${catkin_LIBRARIES}
  )

############## pandora_slam_2d_node ##############

add_executable(${PROJECT_NAME}_pandora_slam_2d_node
  src/pandora_slam_2d_node.cpp)
target_link_libraries(${PROJECT_NAME}_pandora_slam_2d_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_pandora_slam_2d
)
add_dependencies(${PROJECT_NAME}_pandora_slam_2d_node
  state_manager_msgs_gencpp
  )

######################## roslint ###############

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
 RELATIVE ${PROJECT_SOURCE_DIR}
    include/pandora_slam_2d/*.h
    src/*.cpp
    )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
