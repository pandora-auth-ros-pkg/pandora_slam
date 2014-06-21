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
* Author: Chris Zalidis
*********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <string>

namespace pose_estimation_5_dof {


class PoseEstimation {

 public:
  PoseEstimation(int argc, char **argv);
  void serveImuMessage(const sensor_msgs::ImuConstPtr& msg);
  void publishPose(const ros::TimerEvent&);

 private:

  ros::NodeHandle nh_;

  tf::TransformBroadcaster poseBroadcaster_;
  ros::Timer poseBroadcastTimer_;
  ros::Subscriber imuSubscriber_;

  std::string imuTopic_;
  std::string frameFrom_;
  std::string frameTo_;

  double imuYaw_;
  double imuPitch_;
  double imuRoll_;

  double poseFreq_;
};

PoseEstimation::PoseEstimation(int argc, char **argv) {
  
  std::string nodeName("[Pandora pose estimation] : ");
  
  if (nh_.hasParam("/pose_estimation/imu_topic")) {
    nh_.getParam("/pose_estimation/imu_topic", imuTopic_);
  } else {
    ROS_WARN_STREAM(nodeName << 
        "Parameter imu_topic not found. Using Default");
    imuTopic_ = "/sensors/imu";
  }

  if (nh_.hasParam("/pose_estimation/frame_from")) {
    nh_.getParam("/pose_estimation/frame_from", frameFrom_);
  } else {
    ROS_WARN_STREAM(nodeName << 
        "Parameter frame_from not found. Using Default");
    frameFrom_ = "base_footprint";
  }

  if (nh_.hasParam("/pose_estimation/frame_to")) {
    nh_.getParam("/pose_estimation/frame_to", frameTo_);
  } else {
    ROS_WARN_STREAM(nodeName << 
        "Parameter frame_to not found. Using Default");
    frameTo_ = "base_link";
  }

  if (nh_.hasParam("/pose_estimation/pose_freq")) {
    nh_.getParam("/pose_estimation/pose_freq", poseFreq_);
  } else {
    ROS_WARN_STREAM(nodeName << 
        "Parameter pose_freq not found. Using Default");
    poseFreq_ = 5.0;
  }

  imuSubscriber_ = nh_.subscribe(imuTopic_, 
                                  1, 
                                  &PoseEstimation::serveImuMessage, 
                                  this);

  poseBroadcastTimer_ = nh_.createTimer(
      ros::Duration(1.0/poseFreq_), &PoseEstimation::publishPose, this);
  poseBroadcastTimer_.start();

}

void PoseEstimation::serveImuMessage(const sensor_msgs::ImuConstPtr& msg) {

  tf::Matrix3x3 matrix(
    tf::Quaternion(msg->orientation.x, 
                  msg->orientation.y, 
                  msg->orientation.z, 
                  msg->orientation.w));
  matrix.getRPY(imuRoll_, imuPitch_, imuYaw_);
}

void PoseEstimation::publishPose(const ros::TimerEvent&) {
  tf::Vector3 translation(0, 0, 0.144);
  tf::Quaternion rotation;
  rotation.setRPY(imuRoll_, imuPitch_, 0);
  tf::Transform tfTransformFinal(rotation, translation);
  poseBroadcaster_.sendTransform(tf::StampedTransform(tfTransformFinal, 
                                                      ros::Time::now(),
                                                      frameFrom_, 
                                                      frameTo_));
}

} // namespace pose_estimation_5_dof

int main (int argc, char **argv) {
  ros::init(argc, argv, "pose_estimation_node", 
      ros::init_options::NoSigintHandler);
  pose_estimation_5_dof::PoseEstimation poseEstimation(argc, argv);
  ROS_DEBUG("Pandora pose estimation node initialised");
  ros::spin();
  return 0;
}
