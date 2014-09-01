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
#include "pose_estimation_handler/pose_estimation_handler.h"

namespace pandora_slam
{
  PoseEstimationHandler::PoseEstimationHandler()
  {
    slam_2d_frame_id_ = "base_footprint";
    //~ visual_odometry_topic_ = "/vo";
    imu_topic_ = "/sensors/imu";
    if (visual_odometry_topic_ != "")
    {
      visual_odometry_subscriber_ = node_handle_.subscribe(
        visual_odometry_topic_, 1,
        &PoseEstimationHandler::visualOdometryCallback, this);
    }
    if (imu_topic_ != "")
    {
      imu_subscriber_ = node_handle_.subscribe(
        imu_topic_, 1,
        &PoseEstimationHandler::imuCallback, this);
    }
    pose_publisher_ = node_handle_.advertise<
      geometry_msgs::PoseStamped>(
      "/pose_estimation_handler/pose", 5);
  }

  PoseEstimationHandler::~PoseEstimationHandler()
  {
  }

  void PoseEstimationHandler::visualOdometryCallback(
    const nav_msgs::OdometryConstPtr& odom_ptr)
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = odom_ptr->header;
    pose_msg.pose = odom_ptr->pose.pose;
    pose_publisher_.publish(pose_msg);
  }

  void PoseEstimationHandler::imuCallback(
    const sensor_msgs::ImuConstPtr& imu_msg_ptr)
  {
    tf::StampedTransform slam_2d_tf;
    try
    {
      tf_listener.lookupTransform("map", slam_2d_frame_id_,
        ros::Time(0), slam_2d_tf);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_STREAM("Transform error: " << ex.what() <<
        ",quitting callback");
      return;
    }

    tf::Quaternion imu_orientation;
    quaternionMsgToTF(imu_msg_ptr->orientation, imu_orientation);
    tf::Matrix3x3 imu_basis(imu_orientation);
    double dummy_roll, dummy_pitch, dummy_yaw;
    double roll, pitch, yaw;
    imu_basis.getRPY(roll, pitch, dummy_yaw);
    slam_2d_tf.getBasis().getRPY(dummy_roll, dummy_pitch, yaw);
    tf::Matrix3x3 estimation_basis;
    estimation_basis.setRPY(roll, pitch, yaw);

    tf::Transform transform;
    transform.setOrigin(slam_2d_tf.getOrigin());
    transform.setBasis(estimation_basis);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = imu_msg_ptr->header;
    poseTFToMsg(transform, pose_msg.pose);
    pose_publisher_.publish(pose_msg);
  }
}  // namespace pandora_slam

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_estimation_handler");
  pandora_slam::PoseEstimationHandler pose_estimation_handler;

  ros::spin();
  return 0;
}
