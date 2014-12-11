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
* Author: Manos Tsardoulias
*********************************************************************/
#include "pandora_slam_2d/pandora_slam_2d.h"

namespace pandora_slam_2d
{

PandoraSlam::PandoraSlam(int argc, char **argv): crsmSlam_(argc, argv)
{

  state_ = state_manager_msgs::RobotModeMsg::MODE_OFF;
  prevState_ = state_manager_msgs::RobotModeMsg::MODE_OFF;
  clientInitialize();

}


/**
@brief Implemented from StateClient. Called when the robot state changes
@param newState type: int
@return void
**/
void PandoraSlam::startTransition(int newState)
{

  state_ = newState;

  if(state_ == state_manager_msgs::RobotModeMsg::MODE_TERMINATING)
  {
    ROS_ERROR("[Pandora SLAM] Terminating node");
    exit(0);
  }

  bool currStateOn = (state_ != 
                        state_manager_msgs::RobotModeMsg::MODE_OFF);
  bool prevStateOn = (prevState_ != 
                        state_manager_msgs::RobotModeMsg::MODE_OFF);

  if(currStateOn && !prevStateOn) {
    crsmSlam_.startLaserSubscriber();
    crsmSlam_.startOGMPublisher();
    crsmSlam_.startTrajectoryPublisher();
  } else if(!currStateOn && prevStateOn) {
    crsmSlam_.stopLaserSubscriber();
    crsmSlam_.stopOGMPublisher();
    crsmSlam_.stopTrajectoryPublisher();
  }

  prevState_ = state_;

  transitionComplete(state_);
}

/**
@brief Implements the state transition end. Inherited from State client.
@param void
@return void
**/
void PandoraSlam::completeTransition(void)
{
}

} // namespace pandora_slam_2d

