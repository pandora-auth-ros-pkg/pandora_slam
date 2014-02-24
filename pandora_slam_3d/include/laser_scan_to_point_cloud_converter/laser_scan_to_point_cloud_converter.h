#ifndef LASER_SCAN_TO_POINT_CLOUD_CONVERTER_H
#define LASER_SCAN_TO_POINT_CLOUD_CONVERTER_H

#include "ros/ros.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanToPointCloudConverter
{
	private:
		ros::Subscriber _subscriber;
		ros::Publisher _publisher;
	
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
		
	
	public:
		LaserScanToPointCloudConverter();
};


#endif
