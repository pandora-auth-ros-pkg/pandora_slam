/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Evangelos Apostolidis
*********************************************************************/
#include "ros/ros.h"
#include "laser_geometry/laser_geometry.h"

namespace pandora_slam
{
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
    subscriber_ = node_handle_.subscribe("/laser/scan", 1,
      &LaserScanToPointCloudConverter::scanCallback, this);

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
}  // namespace pandora_slam

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_scan_to_point_cloud_converter");
  pandora_slam::LaserScanToPointCloudConverter
    laserScanToPointCloudConverter;

  ros::spin();
  return 0;
}
