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
#ifndef POSE_ESTIMATION_HANDLER_POSE_ESTIMATION_HANDLER_H
#define POSE_ESTIMATION_HANDLER_POSE_ESTIMATION_HANDLER_H

#include "ros/ros.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "utils/timer.h"

namespace pandora_slam
{
  class PoseEstimationHandler
  {
   public:
    PoseEstimationHandler();
    ~PoseEstimationHandler();
   private:
    void visualOdometryCallback(
      const nav_msgs::OdometryConstPtr& odom_ptr);
    void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);
    void cloudCallback(
      const sensor_msgs::PointCloud2ConstPtr& cloud_msg_ptr);

    ros::NodeHandle node_handle_;
    ros::Subscriber visual_odometry_subscriber_;
    ros::Subscriber imu_subscriber_;
    ros::Subscriber cloud_subscriber_;
    ros::Publisher pose_publisher_;
    tf::TransformListener tf_listener;

    std::string slam_2d_frame_id_;
    std::string visual_odometry_topic_;
    std::string imu_topic_;
    std::string cloud_topic_;
  };
}  // namespace pandora_slam

#endif  // POSE_ESTIMATION_HANDLER_POSE_ESTIMATION_HANDLER_H
