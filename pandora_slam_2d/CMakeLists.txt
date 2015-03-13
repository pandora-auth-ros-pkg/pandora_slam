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
    DEPENDS
    CATKIN_DEPENDS
      roscpp
      tf
      state_manager
      diagnostic_updater
      state_manager_msgs
      crsm_slam
      roslint
    INCLUDE_DIRS include
    LIBRARIES # TODO
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

#~ set(CMAKE_BUILD_TYPE Debug)


############ pandora_slam_2d ##############
add_library(pandora_slam_2d src/pandora_slam_2d.cpp)
target_link_libraries(pandora_slam_2d ${catkin_LIBRARIES})

############## pandora_slam_2d_node ##############
add_executable(pandora_slam_2d_node  src/pandora_slam_2d_node.cpp)
target_link_libraries(pandora_slam_2d_node
    ${catkin_LIBRARIES}
    pandora_slam_2d
)
add_dependencies(pandora_slam_2d_node state_manager_msgs_gencpp)


set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line,-build/include_order,-whitespace/blank_line,-whitespace/parens,-whitespace/comments,-whitespace/braces")
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR}
            include/pandora_slam_2d/*.h
            src/*.cpp
            )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

