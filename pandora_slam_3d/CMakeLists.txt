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

#~ laser_scan_to_point_cloud_converter
add_library(${PROJECT_NAME}_laser_scan_to_point_cloud_converter 
            src/laser_scan_to_point_cloud_converter/laser_scan_to_point_cloud_converter.cpp
            )
target_link_libraries(${PROJECT_NAME}_laser_scan_to_point_cloud_converter 
					  ${catkin_LIBRARIES}
					  )


add_executable(${PROJECT_NAME}_laser_scan_to_point_cloud_converter_node 
			   src/laser_scan_to_point_cloud_converter/laser_scan_to_point_cloud_converter_node.cpp
			   )
target_link_libraries(${PROJECT_NAME}_laser_scan_to_point_cloud_converter_node 
					  ${PROJECT_NAME}_laser_scan_to_point_cloud_converter
					  ${catkin_LIBRARIES} 
					  )

#pandora_icp_slam_3d_laser 
#~ add_library(${PROJECT_NAME}_pandora_icp_slam_3d 
			#~ src/pandora_icp_slam_3d_laser/pandora_icp_slam_3d_laser.cpp
			#~ )
#~ target_link_libraries(${PROJECT_NAME}_pandora_icp_slam_3d 
                      #~ ${catkin_LIBRARIES}
                      #~ )
#~ 
#~ add_executable(${PROJECT_NAME}_pandora_icp_slam_3d_node 
			   #~ src/pandora_icp_slam_3d_laser/pandora_icp_slam_3d_laser_node.cpp
			   #~ )
#~ target_link_libraries(${PROJECT_NAME}_pandora_icp_slam_3d_node 
					  #~ ${PROJECT_NAME}_pandora_icp_slam_3d
					  #~ ${catkin_LIBRARIES} 
					  #~ )
					  
					  
#pandora_icp_slam_3d_laser_map 
#~ add_library(${PROJECT_NAME}_pandora_icp_slam_3d 
			#~ src/pandora_icp_slam_3d_laser_map/pandora_icp_slam_3d_laser_map.cpp
			#~ )
#~ target_link_libraries(${PROJECT_NAME}_pandora_icp_slam_3d 
                      #~ ${catkin_LIBRARIES}
                      #~ )
#~ 
#~ add_executable(${PROJECT_NAME}_pandora_icp_slam_3d_node 
			   #~ src/pandora_icp_slam_3d_laser_map/pandora_icp_slam_3d_laser_map_node.cpp
			   #~ )
#~ target_link_libraries(${PROJECT_NAME}_pandora_icp_slam_3d_node 
					  #~ ${PROJECT_NAME}_pandora_icp_slam_3d
					  #~ ${catkin_LIBRARIES} 
					  #~ )
					  
					  
#pandora_icp_slam_3d_kinect 
#~ add_library(${PROJECT_NAME}_pandora_icp_slam_3d 
			#~ src/pandora_icp_slam_3d_kinect/pandora_icp_slam_3d_kinect.cpp
			#~ )
#~ target_link_libraries(${PROJECT_NAME}_pandora_icp_slam_3d 
                      #~ ${catkin_LIBRARIES}
                      #~ )
#~ 
#~ add_executable(${PROJECT_NAME}_pandora_icp_slam_3d_node 
			   #~ src/pandora_icp_slam_3d_kinect/pandora_icp_slam_3d_kinect_node.cpp
			   #~ )
#~ target_link_libraries(${PROJECT_NAME}_pandora_icp_slam_3d_node 
					  #~ ${PROJECT_NAME}_pandora_icp_slam_3d
					  #~ ${catkin_LIBRARIES} 
					  #~ )
					  
					  
#pandora_icp_slam_3d_kinect_map 
add_library(${PROJECT_NAME}_pandora_icp_slam_3d 
			src/pandora_icp_slam_3d_kinect_map/pandora_icp_slam_3d_kinect_map.cpp
			)
target_link_libraries(${PROJECT_NAME}_pandora_icp_slam_3d 
                      ${catkin_LIBRARIES}
                      )

add_executable(${PROJECT_NAME}_pandora_icp_slam_3d_node 
			   src/pandora_icp_slam_3d_kinect_map/pandora_icp_slam_3d_kinect_map_node.cpp
			   )
target_link_libraries(${PROJECT_NAME}_pandora_icp_slam_3d_node 
					  ${PROJECT_NAME}_pandora_icp_slam_3d
					  ${catkin_LIBRARIES} 
					  )
