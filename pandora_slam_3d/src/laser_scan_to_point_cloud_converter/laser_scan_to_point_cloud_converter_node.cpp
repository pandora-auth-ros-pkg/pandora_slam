#include "laser_scan_to_point_cloud_converter/laser_scan_to_point_cloud_converter.h"

int main (int argc, char **argv){
	
	ros::init(argc,argv,"laser_scan_to_point_cloud_converter_node");
	LaserScanToPointCloudConverter laserScanToPointCloudConverter;
	
	ros::spin();
	return 0;
}
