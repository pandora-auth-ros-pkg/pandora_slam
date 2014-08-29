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
#ifndef MATCHING_OCTOMAP_SERVER_MATCHING_OCTOMAP_SERVER_H
#define MATCHING_OCTOMAP_SERVER_MATCHING_OCTOMAP_SERVER_H

#include "octomap_server/OctomapServer.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "matching_octomap_server/randomized_transform.h"
#include "utils/timer.h"

typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
  nav_msgs::Odometry> PCSyncPolicy;

namespace pandora_slam
{
  class MatchingOctomapServer : public octomap_server::OctomapServer
  {
   public:
    MatchingOctomapServer();
    ~MatchingOctomapServer();
   private:
    void matchCloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& full_cloud,
      const sensor_msgs::PointCloud2::ConstPtr& subsampled_cloud,
      const nav_msgs::OdometryConstPtr& odom_ptr);

    void filterAndPublishCloud(
      const sensor_msgs::PointCloud2::ConstPtr& input_cloud_ptr);

    tf::Transform previous_odom_;
    tf::Transform previous_tf_;
    tf::TransformBroadcaster tf_broadcaster_;

    message_filters::Subscriber<sensor_msgs::PointCloud2>*
      point_cloud_subscriber_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>*
      subsampled_cloud_subscriber_;
    message_filters::Subscriber<nav_msgs::Odometry>*
      odom_subscriber_;
    message_filters::Synchronizer<PCSyncPolicy>* synchronizer_;

    ros::Publisher cloud_publisher_;

    double voxel_size_;
  };
}  // namespace pandora_slam

#endif  // MATCHING_OCTOMAP_SERVER_MATCHING_OCTOMAP_SERVER_H
