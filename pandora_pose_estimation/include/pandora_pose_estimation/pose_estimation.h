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

#ifndef PANDORA_POSE_ESTIMATION_POSE_ESTIMATION_H
#define PANDORA_POSE_ESTIMATION_POSE_ESTIMATION_H

#include <cmath>
#include <string>
#include <deque>
#include <boost/math/constants/constants.hpp>

#include "ros/time.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/Imu.h"

//DEBUG z
#include "std_msgs/Float64MultiArray.h"

#include "state_manager/state_client.h"
#include "state_manager_msgs/RobotModeMsg.h"

namespace pandora_pose_estimation
{
  class PoseEstimation: public StateClient
  {
   public:
    PoseEstimation(const std::string& ns);
    void serveImuMessage(const sensor_msgs::ImuConstPtr& msg);
    void publishPose(const ros::TimerEvent&);

    void startTransition(int newState);
    void completeTransition();

   private:
    double findDz(double dx, double dy);
    double probFilt(double expectation, double measurement);
    double linInterp(double past, double current, int steps);
    double dz(double dx, double dy, double roll, double pitch);

   private:
    ros::NodeHandle nh_;

    tf::TransformBroadcaster poseBroadcaster_;
    tf::TransformListener poseListener_;
    ros::Timer poseBroadcastTimer_;
    ros::Subscriber imuSubscriber_;
    std::string imuTopic_;

    std::string frameMap_;
    std::string frameFootprint_;
    std::string frameFootprintElevated_;
    std::string frameStabilized_;
    std::string frameLink_;

    tf::Transform previousTf_;

    double latestYaw_, latestPitch_, latestRoll_;
    double *onExpFilt_;

    //!< IMU && SLAM history
    std::deque<double> pitchQ_, rollQ_, dxQ_, dyQ_;

    int currentState_;

    double POSE_FREQ, FLAT_TO_AXES, PROBFILT_DECAY;
    int FILTER_LENGTH;

    //DEBUG z
    ros::Publisher zTransPub_;
    ros::Publisher imuPub_;
  };

}  // namespace pandora_pose_estimation

#endif  // PANDORA_POSE_ESTIMATION_POSE_ESTIMATION_H
