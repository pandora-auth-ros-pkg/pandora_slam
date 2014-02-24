#include "laser_scan_to_point_cloud_converter/laser_scan_to_point_cloud_converter.h"

LaserScanToPointCloudConverter::LaserScanToPointCloudConverter(){
	ros::NodeHandle nodeHandle;
	
	_subscriber = nodeHandle.subscribe("/slam/scan",1, &LaserScanToPointCloudConverter::scanCallback,this);
	
	_publisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("/laser/point_cloud", 5);
	
	
}



void LaserScanToPointCloudConverter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	laser_geometry::LaserProjection projector_;
	sensor_msgs::PointCloud2 cloud;
	projector_.projectLaser(*scan_in, cloud);
	_publisher.publish(cloud);

}
