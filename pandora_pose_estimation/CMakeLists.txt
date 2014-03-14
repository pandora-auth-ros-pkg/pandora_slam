cmake_minimum_required(VERSION 2.8.3)
project(pandora_pose_estimation)
find_package(catkin REQUIRED COMPONENTS
      roscpp
      tf
      sensor_msgs
      roslint
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pandora_pose_estimation
  CATKIN_DEPENDS 
    roscpp
    tf
    sensor_msgs
    roslint
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS}
)

############## pose_estimation_5_dof_node ##############
add_executable(pose_estimation_node
  src/pose_estimation_node.cpp)
target_link_libraries(pose_estimation_node ${catkin_LIBRARIES})


set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line,-build/include_order,-whitespace/blank_line,-whitespace/parens,-whitespace/comments,-whitespace/braces")
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
     RELATIVE ${PROJECT_SOURCE_DIR} 
            src/*.cpp 
            )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
