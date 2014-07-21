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
#include <tf/transform_listener.h>
#include "pcl_ros/impl/transforms.hpp"

namespace pandora_slam
{
  class PointCloudAggregator
  {
   public:
    PointCloudAggregator();
   private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in);

    ros::NodeHandle node_handle_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    tf::TransformListener tf_listener_;
    ros::Time last_time_published_;
    ros::Duration publish_period_;
    sensor_msgs::PointCloud2 aggregated_cloud_;
  };

  PointCloudAggregator::PointCloudAggregator()
  {
    subscriber_ = node_handle_.subscribe("/laser/point_cloud", 1,
      &PointCloudAggregator::cloudCallback, this);

    publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
      "/laser/aggregated_point_cloud", 5);

    last_time_published_ = ros::Time::now();
    publish_period_ = ros::Duration(2);
  }

  void PointCloudAggregator::cloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_in)
  {
    sensor_msgs::PointCloud2 transformed_cloud_in;
    if (!pcl_ros::transformPointCloud(
      "laser_servo_link", *cloud_in, transformed_cloud_in, tf_listener_))
    {
      return;
    }
    if (aggregated_cloud_.width == 0)
    {
      aggregated_cloud_ = transformed_cloud_in;
      return;
    }

    for (int ii = 0; ii < cloud_in->data.size(); ii++)
    {
      aggregated_cloud_.data.push_back(transformed_cloud_in.data[ii]);
    }
    aggregated_cloud_.width += transformed_cloud_in.width;

    if (ros::Time::now() - last_time_published_ > publish_period_)
    {
      publisher_.publish(aggregated_cloud_);
      aggregated_cloud_.data.clear();
      aggregated_cloud_.width = 0;
      last_time_published_ = ros::Time::now();
    }
  }
}  // namespace pandora_slam


int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_aggregator");
  pandora_slam::PointCloudAggregator point_cloud_aggregator;

  ros::spin();
  return 0;
}
