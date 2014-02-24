  /** 
  * File Description main - Initialises SLAM module
  * Contents:
		main
  * Author: SLAM team
  */

#include "pandora_slam_2d/pandora_slam_2d.h"


//!< Main function of the node
int main (int argc, char **argv){
	ros::init(argc,argv,"pandora_slam_2d_node",ros::init_options::NoSigintHandler);	
	pandora_slam_2d::PandoraSlam slam(argc,argv);
	ROS_DEBUG("Pandora Slam node initialised");
	ros::spin();
	return 0;
}
