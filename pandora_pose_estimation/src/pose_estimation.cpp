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
 * Authors:
 *   Author Name <author's email>
 *********************************************************************/

#include "pose_estimation/pose_estimation.h"

namespace pose_estimation
{

  PoseEstimation::PoseEstimation(int argc, char **argv)
  {
    std::string nodeName("[Pandora pose estimation] : ");

    if (nh_.hasParam("/pose_estimation/imu_topic")) {
      nh_.getParam("/pose_estimation/imu_topic", imuTopic_);
    } else {
      ROS_WARN_STREAM(nodeName <<
          "Parameter imu_topic not found. Using Default");
      imuTopic_ = "/sensors/imu";
    }

    if (nh_.hasParam("/pose_estimation/frame_map")) {
      nh_.getParam("/pose_estimation/frame_map", frameMap_);
    } else {
      ROS_WARN_STREAM(nodeName <<
          "Parameter frame_map not found. Using Default");
      frameMap_ = "/map";
    }

    if (nh_.hasParam("/pose_estimation/frame_flat")) {
      nh_.getParam("/pose_estimation/frame_flat", frameFlat_);
    } else {
      ROS_WARN_STREAM(nodeName <<
          "Parameter frame_flat not found. Using Default");
      frameFlat_ = "base_flat_footprint";
    }

    if (nh_.hasParam("/pose_estimation/frame_footprint")) {
      nh_.getParam("/pose_estimation/frame_footprint", frameFootprint_);
    } else {
      ROS_WARN_STREAM(nodeName <<
          "Parameter frame_footprint not found. Using Default");
      frameFootprint_ = "base_footprint";
    }

    if (nh_.hasParam("/pose_estimation/frame_stabilized")) {
      nh_.getParam("/pose_estimation/frame_stabilized", frameStabilized_);
    } else {
      ROS_WARN_STREAM(nodeName <<
          "Parameter frame_stabilized not found. Using Default");
      frameStabilized_ = "base_stabilized";
    }

    if (nh_.hasParam("/pose_estimation/frame_link")) {
      nh_.getParam("/pose_estimation/frame_link", frameLink_);
    } else {
      ROS_WARN_STREAM(nodeName <<
          "Parameter frame_link not found. Using Default");
      frameLink_ = "base_link";
    }

    if (nh_.hasParam("/pose_estimation/pose_freq")) {
      nh_.getParam("/pose_estimation/pose_freq", poseFreq_);
    } else {
      ROS_WARN_STREAM(nodeName <<
          "Parameter pose_freq not found. Using Default");
      poseFreq_ = 5.0;
    }

    if (nh_.hasParam("/pose_estimation/distance_to_axes")) {
      nh_.getParam("/pose_estimation/distance_to_axes", FLAT_TO_AXES);
    } else {
      ROS_WARN_STREAM(nodeName <<
          "Parameter flat_to_axes not found. Using Default");
      FLAT_TO_AXES = 0.145;
    }

    // Maybe it's something else relatively to /map frame
    previousOrigin_.setZero();

    imuSubscriber_ = nh_.subscribe(imuTopic_,
                                    1,
                                    &PoseEstimation::serveImuMessage,
                                    this);

    poseBroadcastTimer_ = nh_.createTimer(
        ros::Duration(1.0/poseFreq_), &PoseEstimation::publishPose, this);
    poseBroadcastTimer_.start();
  }

  void PoseEstimation::serveImuMessage(const sensor_msgs::ImuConstPtr& msg)
  {
    tf::Matrix3x3 matrix(
      tf::Quaternion(msg->orientation.x,
                    msg->orientation.y,
                    msg->orientation.z,
                    msg->orientation.w));
    matrix.getRPY(imuRoll_, imuPitch_, imuYaw_);
  }

  void PoseEstimation::publishPose(const ros::TimerEvent&)
  {
    tf::Quaternion rotationZero;
    rotationZero.setRPY(0, 0, 0);

    // Get frame flat
    tf::StampedTransform intermediateTf;
    poseListener_.lookupTransform(frameFlat_, frameMap_,
        ros::Time::now(), intermediateTf);
    tf::Vector3 origin;
    tfScalar pitch, roll, yaw;
    intermediateTf.getBasis().getRPY(roll, pitch, yaw);
    origin = intermediateTf.getOrigin();

    // Get difference in x and y
    double dx, dy, final_z;
    dx = origin.getX() - previousOrigin_.getX();
    dy = origin.getY() - previousOrigin_.getY();
    // Find difference in z
    final_z = findDz(dx, dy, imuRoll_, imuPitch_) + previousOrigin_.getZ();
    // Update previousOrigin_
    previousOrigin_ = origin;
    previousOrigin_.setZ(final_z);
    // Broadcast updated footprint transform
    tf::Vector3 translationZ(0, 0, final_z);
    tf::Transform tfDz(rotationZero, translationZ);
    poseBroadcaster_.sendTransform(tf::StampedTransform(tfDz,
                                                        ros::Time::now(),
                                                        frameFlat_,
                                                        frameFootprint_));
    // Broadcast updated base stabilized
    tf::Vector3 translationVert(0, 0, FLAT_TO_AXES);
    tf::Transform tfTransformFinal(rotationZero, translationVert);
    poseBroadcaster_.sendTransform(tf::StampedTransform(tfTransformFinal,
                                                        ros::Time::now(),
                                                        frameFootprint_,
                                                        frameStabilized_));

    tf::Vector3 translationZero(0, 0, 0);
    tf::Quaternion rotation;
    rotation.setRPY(imuRoll_, imuPitch_, 0);
    // base_stabilized -> base_link
    tf::Transform tfTransformFinal2(rotation, translationZero);
    poseBroadcaster_.sendTransform(tf::StampedTransform(tfTransformFinal2,
                                                        ros::Time::now(),
                                                        frameStabilized_,
                                                        frameLink_));
  }

  /** @brief PoseEstimation::findDz Computes differential vertical translation in global coords
   * @param dx Horizontal translation in global coords
   * @param dy Horizontal translation in global coords
   * @param roll Rotation around x axis in DEGREES
   * @param pitch Rotation around y axis in DEGREES
   */
  double PoseEstimation::findDz(double dx, double dy, double roll, double pitch)
  {
    return -tan(pitch*PI/180.0)/cos(roll*PI/180.0)*dx - tan(roll)*dy;
    //return -tan(pitch)/cos(roll)*dx -tan(roll)*dy;
  }

} // namespace pose_estimation
