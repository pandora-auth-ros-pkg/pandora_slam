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

#include <string>
#include <cmath>
#include <boost/math/constants/constants.hpp>

#include "state_manager_msgs/RobotModeMsg.h"

#include "pandora_pose_estimation/pose_estimation.h"

namespace pandora_pose_estimation
{

  PoseEstimation::PoseEstimation(const std::string& ns): nh_(ns)
  {
    ROS_INFO("[%s] : Constructing PoseEstimation...!",
        nh_.getNamespace().c_str());

    nh_.param<std::string>("imu_topic", imuTopic_, "/sensors/imu");
    nh_.param<std::string>("frame_map", frameMap_, "/world");
    nh_.param<std::string>("frame_flat", frameFlat_, "/base_footprint");
    nh_.param<std::string>("frame_footprint", frameFootprint_, "/base_footprint_elevated");
    nh_.param<std::string>("frame_stabilized", frameStabilized_, "/base_stabilized");
    nh_.param<std::string>("frame_link", frameLink_, "/base_link");
    nh_.param<double>("pose_frequency", POSE_FREQ, 5.0);
    nh_.param<double>("distance_to_axes", FLAT_TO_AXES, 0.145);

    // Maybe it's something else relatively to /map frame
    previousOrigin_.setZero();

    imuSubscriber_ = nh_.subscribe(imuTopic_, 1,
        &PoseEstimation::serveImuMessage, this);

    poseBroadcastTimer_ = nh_.createTimer(
        ros::Duration(1.0/POSE_FREQ), &PoseEstimation::publishPose, this);
    currentState_ = state_manager_msgs::RobotModeMsg::MODE_OFF;
    clientInitialize();
    poseBroadcastTimer_.start();
    ROS_INFO("[%s] : Constructed!", nh_.getNamespace().c_str());
  }

  void PoseEstimation::startTransition(int newState)
  {
    currentState_ = newState;
    transitionComplete(currentState_);
  }

  void PoseEstimation::completeTransition()
  {
  }

  void PoseEstimation::serveImuMessage(const sensor_msgs::ImuConstPtr& msg)
  {
    tf::Matrix3x3 matrix(tf::Quaternion(
          msg->orientation.x,
          msg->orientation.y,
          msg->orientation.z,
          msg->orientation.w));
    matrix.getRPY(imuRoll_, imuPitch_, imuYaw_);
  }

  void PoseEstimation::publishPose(const ros::TimerEvent&)
  {
    tf::Quaternion rotationZero;
    rotationZero.setRPY(0, 0, 0);
    ros::Time mostRecentSlam = ros::Time::now();

    // Get frame flat
    if (currentState_ != state_manager_msgs::RobotModeMsg::MODE_OFF) {
      ROS_INFO("[%s] : Get frame flat...", nh_.getNamespace().c_str());
      tf::StampedTransform intermediateTf;
      poseListener_.waitForTransform(frameFlat_, frameMap_,
          mostRecentSlam, ros::Duration(1.0/POSE_FREQ));
      poseListener_.lookupTransform(frameFlat_, frameMap_,
          mostRecentSlam, intermediateTf);
      ROS_INFO("[%s] : Got it!", nh_.getNamespace().c_str());
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
      ROS_INFO("[%s] : Broadcast footprint tf...", nh_.getNamespace().c_str());
      tf::Vector3 translationZ(0, 0, final_z);
      tf::Transform tfDz(rotationZero, translationZ);
      poseBroadcaster_.sendTransform(tf::StampedTransform(tfDz,
                                                          mostRecentSlam,
                                                          frameFlat_,
                                                          frameFootprint_));
    }
    else {
      tf::Transform tfDz(rotationZero, tf::Vector3(0, 0, 0));
      poseBroadcaster_.sendTransform(tf::StampedTransform(tfDz,
                                                          mostRecentSlam,
                                                          frameFlat_,
                                                          frameFootprint_));
    }

    // Broadcast updated base stabilized
    ROS_INFO("[%s] : Broadcast base_stab tf...", nh_.getNamespace().c_str());
    tf::Vector3 translationVert(0, 0, FLAT_TO_AXES);
    tf::Transform tfTransformFinal(rotationZero, translationVert);
    poseBroadcaster_.sendTransform(tf::StampedTransform(tfTransformFinal,
                                                        mostRecentSlam,
                                                        frameFootprint_,
                                                        frameStabilized_));

    tf::Vector3 translationZero(0, 0, 0);
    tf::Quaternion rotation;
    rotation.setRPY(imuRoll_, imuPitch_, 0);
    // base_stabilized -> base_link
    tf::Transform tfTransformFinal2(rotation, translationZero);
    poseBroadcaster_.sendTransform(tf::StampedTransform(tfTransformFinal2,
                                                        mostRecentSlam,
                                                        frameStabilized_,
                                                        frameLink_));
  }

  /** @brief PoseEstimation::findDz Computes differential vertical translation in global coords
   * @param dx Horizontal translation in global coords
   * @param dy Horizontal translation in global coords
   * @param roll Rotation around x axis in RAD
   * @param pitch Rotation around y axis in RAD
   */
  double PoseEstimation::findDz(double dx, double dy, double roll, double pitch)
  {
    ROS_INFO("[%s] : FindDz...", nh_.getNamespace().c_str());
    return tan(pitch)/cos(roll)*dx -tan(roll)*dy;
  }

} // namespace pandora_pose_estimation
