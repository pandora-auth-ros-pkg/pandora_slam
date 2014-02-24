#include "pandora_icp_slam_3d_laser/pandora_icp_slam_3d_laser.h"

int main (int argc, char **argv){
	
	ros::init(argc,argv,"pandora_icp_slam_3d_node");
	
	pandora_slam_3d::PandoraIcpSlam3d pandoraIcpSlam3d;
	
	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
	spinner.spin(); // spin() will not return until the node has been shutdown
	return 0;
}
