# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pandora_slam_2d)
find_package(catkin REQUIRED COMPONENTS roscpp 
	  tf 
	  state_manager
	  diagnostic_updater
	  state_manager_communications
	  crsm_slam
)


catkin_package(
    DEPENDS   
    CATKIN-DEPENDS
      tf 
      state_manager
      diagnostic_updater
      state_manager_communications
      crsm_slam
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories(include
  ${catkin_INCLUDE_DIRS} 
)

set(CMAKE_BUILD_TYPE Debug)

link_directories(${catkin_LIBRARY_DIRS})


############ pandora_slam_2d ##############
add_library(pandora_slam_2d src/pandora_slam_2d.cpp)
target_link_libraries(pandora_slam_2d ${catkin_LIBRARIES} 
crsm_slam # remove when exported from crsm_slam
)

############## pandora_slam_2d_node ##############
add_executable(pandora_slam_2d_node  src/pandora_slam_2d_node.cpp)
target_link_libraries(pandora_slam_2d_node 
    ${catkin_LIBRARIES} 
    pandora_slam_2d
)
add_dependencies(pandora_slam_2d_node state_manager_communications_gencpp)
