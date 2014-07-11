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
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
namespace pandora_slam
{
  class PointCloudSubsampler
  {
   public:
    PointCloudSubsampler();
   private:
    void cloudCallback(
      const PointCloud::ConstPtr& cloud_in);

    ros::NodeHandle node_handle_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    pcl::visualization::RangeImageVisualizer range_image_widget_;
  };

  PointCloudSubsampler::PointCloudSubsampler() :
    range_image_widget_("Range image")
  {
    subscriber_ = node_handle_.subscribe< PointCloud >(
      "/kinect/depth_registered/points", 1,
      &PointCloudSubsampler::cloudCallback,this);

    publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
      "/kinect/depth_registered/points/subsampled", 5);
  }

  void PointCloudSubsampler::cloudCallback(
    const PointCloud::ConstPtr& cloud_in)
  {
    PointCloud cloud(*cloud_in);
    // Correct dimensions published by gazebo plugin
    cloud.width = 640;
    cloud.height = 480;
    pcl::RangeImage range_image;
    range_image.createFromPointCloud(
      cloud,
      pcl::deg2rad (0.5f),
      pcl::deg2rad (360.0f),
      pcl::deg2rad (180.0f),
      Eigen::Affine3f::Identity (),
      pcl::RangeImage::CAMERA_FRAME,
      0.0f,
      0.0f,
      0);

    range_image_widget_.showRangeImage (range_image);
  }
}  // namespace pandora_slam


int main (int argc, char **argv)
{
  ros::init(argc,argv,"point_cloud_aggregator");
  pandora_slam::PointCloudSubsampler point_cloud_subsampler;

  ros::spin();
  return 0;
}
