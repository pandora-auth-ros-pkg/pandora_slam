#include "ros/ros.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanToPointCloudConverter
{
 public:
  LaserScanToPointCloudConverter();
 private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);

  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
};

LaserScanToPointCloudConverter::LaserScanToPointCloudConverter()
{
  subscriber_ = node_handle_.subscribe("/slam/scan",1,
    &LaserScanToPointCloudConverter::scanCallback,this);

  publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
    "/laser/point_cloud", 5);
}

void LaserScanToPointCloudConverter::scanCallback(
  const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud2 cloud;
  projector_.projectLaser(*scan_in, cloud);
  publisher_.publish(cloud);
}

int main (int argc, char **argv)
{
  ros::init(argc,argv,"laser_scan_to_point_cloud_converter");
  LaserScanToPointCloudConverter laserScanToPointCloudConverter;

  ros::spin();
  return 0;
}
