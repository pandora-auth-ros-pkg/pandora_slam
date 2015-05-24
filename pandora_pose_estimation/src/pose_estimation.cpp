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

#include <deque>
#include <string>
#include "pandora_pose_estimation/pose_estimation.h"

namespace pandora_pose_estimation
{

  PoseEstimation::PoseEstimation(const std::string& ns): nh_(ns)
  {
    ROS_INFO("[%s] : Constructing PoseEstimation...!",
        nh_.getNamespace().c_str());

    nh_.param<std::string>("imu_topic", imuTopic_, "/sensors/imu");
    nh_.param<std::string>("frame_map", frameMap_, "/world");
    nh_.param<std::string>("frame_footprint", frameFootprint_, "/base_footprint");
    nh_.param<std::string>("frame_footprint_elevated", frameFootprintElevated_, "/base_footprint_elevated");
    nh_.param<std::string>("frame_stabilized", frameStabilized_, "/base_stabilized");
    nh_.param<std::string>("frame_link", frameLink_, "/base_link");
    nh_.param<double>("pose_frequency", POSE_FREQ, 5.0);
    nh_.param<double>("distance_to_axes", FLAT_TO_AXES, 0.145);
    nh_.param<int>("SLAM_exp_filter_length", FILTER_LENGTH, 8);
    nh_.param<int>("IMU_interpolation_steps", IMU_INTERP_STEPS, 6);

    // Initialize exponential decay filter
    double rate, normalizeSum= 0;
    onExpFilt_= new double[FILTER_LENGTH];
    for(rate= 0.05; exp((1-FILTER_LENGTH)*rate)< 0.1; rate+= 0.005) {};
    rate-= 0.005;
    for(int i= 0; i< FILTER_LENGTH; i++){
      onExpFilt_[i]= exp(-i*rate);
      normalizeSum+= onExpFilt_[i];
    }
    for(int i= 0; i< FILTER_LENGTH; i++){
      onExpFilt_[i]/= normalizeSum;
    }

    // Maybe it's something else relatively to /map frame
    previousTf_.setIdentity();

    imuSubscriber_ = nh_.subscribe(imuTopic_, 1,
        &PoseEstimation::serveImuMessage, this);

    //DEBUG z
    zTransPub_= nh_.advertise<std_msgs::Float64MultiArray>("/slam/z_est", 10);
    imuPub_= nh_.advertise<std_msgs::Float64MultiArray>("/slam/imu", 10);

    poseBroadcastTimer_ = nh_.createTimer(
        ros::Duration(1.0/POSE_FREQ), &PoseEstimation::publishPose, this);
    currentState_ = state_manager_msgs::RobotModeMsg::MODE_OFF;
    clientInitialize();
    poseBroadcastTimer_.start();
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
    matrix.getRPY(latestRoll_, latestPitch_, latestYaw_);
    pitchQ_.push_back(latestPitch_);
    rollQ_.push_back(latestRoll_);

    //DEBUG z --> Publish IMU RPY
    std_msgs::Float64MultiArray imuMsg;
    std_msgs::MultiArrayLayout layout; std_msgs::MultiArrayDimension dim;
    dim.label= "1"; dim.size= 4; dim.stride= 4;
    layout.dim.push_back(/*MultiArrayDimension*/dim);
    imuMsg.layout= layout;
    imuMsg.data.push_back(/*float*/latestRoll_);
    imuMsg.data.push_back(/*float*/latestPitch_);
    imuMsg.data.push_back(/*float*/latestYaw_);
    imuMsg.data.push_back(ros::Time::now().toSec());
    imuPub_.publish(imuMsg);
  }

  void PoseEstimation::publishPose(const ros::TimerEvent&)
  {
    /*static long fastDuration, slowDuration;
    static int fastN, slowN;
    long beginTime= ros::Time::now().toNSec();*/

    tf::Quaternion rotationZero;
    rotationZero.setRPY(0, 0, 0);

    // Get frame flat
    if (currentState_ != state_manager_msgs::RobotModeMsg::MODE_OFF) {
      tf::StampedTransform intermediateTf;
      std::string tfError;
      poseListener_.waitForTransform(frameFootprint_, frameMap_,
          ros::Time::now(), ros::Duration(1.0), ros::Duration(1/POSE_FREQ),
          &tfError);
      if (!tfError.empty()) return;
      poseListener_.lookupTransform(frameFootprint_, frameMap_,
          ros::Time(0), intermediateTf);
      
      tf::Vector3 previousOrigin= previousTf_.getOrigin(),
                  origin = intermediateTf.getOrigin();
      // Get difference in x and y
      double dx, dy, final_z;
      dx = origin.getX() - previousOrigin.getX();
      dy = origin.getY() - previousOrigin.getY();

      tfScalar roll, pitch, yaw;
      intermediateTf.getBasis().getRPY(roll, pitch, yaw);
      // Find difference in z
      // TODO(lynx): interpolation steps as parameter
      final_z = findDz(dx, dy) + previousOrigin.getZ();
      // Update previousOrigin
      previousTf_.setOrigin(tf::Vector3(origin.getX(), origin.getY(), final_z));
      previousTf_.setRotation(intermediateTf.getRotation());
      // Broadcast updated footprint transform
      //tf::Vector3 translationZ(0, 0, final_z);

      // For testing purposes: keep z in internal state without affecting others
      tf::Vector3 translationZ(0, 0, 0);
      tf::Transform tfDz(rotationZero, translationZ);
      poseBroadcaster_.sendTransform(tf::StampedTransform(tfDz,
                                                      ros::Time::now(),
                                                      frameFootprint_,
                                                      frameFootprintElevated_));

      //DEBUG z
      std_msgs::Float64MultiArray zMsg;
      std_msgs::MultiArrayLayout layout; std_msgs::MultiArrayDimension dim;
      dim.label= "1"; dim.size= 6; dim.stride= 6;
      layout.dim.push_back(/*MultiArrayDimension*/dim);
      zMsg.layout= layout;
      zMsg.data.push_back(/*float*/final_z);
      zMsg.data.push_back(/*float*/dx);
      zMsg.data.push_back(/*float*/dy);
      zMsg.data.push_back(/*float*/latestRoll_);
      zMsg.data.push_back(/*float*/latestPitch_);
      zMsg.data.push_back(ros::Time::now().toSec());
      zTransPub_.publish(zMsg);
    } else {
      tf::Transform tfDz(rotationZero, tf::Vector3(0, 0, 0));
      poseBroadcaster_.sendTransform(tf::StampedTransform(tfDz,
                                                      ros::Time::now(),
                                                      frameFootprint_,
                                                      frameFootprintElevated_));
    }

    // Broadcast updated base stabilized
    tf::Vector3 translationVert(0, 0, FLAT_TO_AXES);
    tf::Transform tfTransformFinal(rotationZero, translationVert);
    poseBroadcaster_.sendTransform(tf::StampedTransform(tfTransformFinal,
                                                      ros::Time::now(),
                                                      frameFootprintElevated_,
                                                      frameStabilized_));

    tf::Vector3 translationZero(0, 0, 0);
    tf::Quaternion rotation;
    rotation.setRPY(latestRoll_, latestPitch_, 0);
    // base_stabilized -> base_link
    tf::Transform tfTransformFinal2(rotation, translationZero);
    poseBroadcaster_.sendTransform(tf::StampedTransform(tfTransformFinal2,
                                                      ros::Time::now(),
                                                      frameStabilized_,
                                                      frameLink_));
    //Measure exec time and INFO rate
    /*
    slowDuration+= ros::Time::now().toNSec()-beginTime;
    slowN++;
    if( slowN%(25)==0 ){
      ROS_INFO("[PoseEstimation]: slow rate = %f, period = %f", 
               ((double)slowN/(double)slowDuration)*1E+9, ((double)slowDuration/(double)slowN)*1E-9);
      slowDuration= 0;
      slowN= 0;
    }
    */
  }

  /**
   * @details Find dz by supersampling total displacement (dx,dy), by assuming
   constant velocity, to initially match the number of extra IMU measurements.
   Then supersample each IMU measurement (and consequently again supersample 
   the displacement). Possible to replace unlikely IMU values with a simple
   filter if we extrapolate from 2 previous IMU values
   * @param imuInterpSteps Must be >0
   */
  double PoseEstimation::findDz(double dx, double dy)
  {
    int totalSteps= (pitchQ_.size()-1)*IMU_INTERP_STEPS;
    double sum_dz= 0, pastRoll, pastPitch, stepRoll, stepPitch;

    // Newest measurements at the front of the queue!
    dxQ_.push_front(dx);
    dyQ_.push_front(dy);
    // Accumulators
    double dxAccum= 0, dyAccum= 0;
    std::deque<double>::iterator ix= dxQ_.begin(), iy= dyQ_.begin();
    for(int i= 0; i< FILTER_LENGTH && ix!= dxQ_.end(); i++, ix++, iy++)
      dxAccum+= (*ix)*onExpFilt_[i], dyAccum+= (*iy)*onExpFilt_[i];
    dxQ_.front()= dxAccum, dyQ_.front()= dyAccum;
    //Leave 1 element in queue as starting point for next loop
    //TODO(lynx): exp filter IMU
    while(pitchQ_.size()> 1){
      pastRoll= rollQ_.front(); rollQ_.pop_front();
      pastPitch= pitchQ_.front(); pitchQ_.pop_front();
      stepRoll= linInterp(pastRoll, rollQ_.front(), IMU_INTERP_STEPS);
      stepPitch= linInterp(pastPitch, pitchQ_.front(), IMU_INTERP_STEPS);
      for(int i= 0; i< IMU_INTERP_STEPS; i++)
        sum_dz+= dz(dxQ_.front()/totalSteps, dyQ_.front()/totalSteps,
                    pastRoll+i*stepRoll, pastPitch+i*stepPitch);  
    }
    dxQ_.resize(FILTER_LENGTH);
    dyQ_.resize(FILTER_LENGTH);
    
    return sum_dz;
  }

  /**
   * @brief Linearly interpolates between past--current vals
   * @returns Interpolation step
   */
  double PoseEstimation::linInterp(double past, double current, int steps)
  {
    return (current-past)/steps;
  }

  /** @brief PoseEstimation::dz Computes differential vertical translation in global coords
   * @param dx Horizontal translation in global coords
   * @param dy Horizontal translation in global coords
   * @param roll Rotation around x axis in RAD
   * @param pitch Rotation around y axis in RAD
   */
  double PoseEstimation::dz(double dx, double dy, double roll, double pitch)
  {
    //the 0.9 factor might help reduce quantization error?
    return +tan(pitch)/cos(roll)*dx*0.9 -tan(roll)*dy;
  }

} // namespace pandora_pose_estimation
