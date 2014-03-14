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

#ifndef PANDORA_SLAM_2D_PANDORA_SLAM_2D_H
#define PANDORA_SLAM_2D_PANDORA_SLAM_2D_H

#include "state_manager/state_client.h"
#include <crsm_slam/crsm_slam.h>

namespace pandora_slam_2d
{

/**
 @class Slam
 @brief The main slam class. Contains the main functionalities of slam.
 **/
class PandoraSlam : public StateClient
{
 private:

  int state_;
  int prevState_;

  crsm_slam::CrsmSlam crsmSlam_;

 public:
  /**
  @brief Default costructor
  @param argc [int] The number of input arguments
  @param argv [char **] The input arguments
  @return void
  **/
  PandoraSlam(int argc, char **argv);

  /**
  @brief Implemented from StateClient. Called when the robot state changes
  @param newState type: int
  @return void
  **/
  void startTransition(int newState);

  /**
  @brief Implements the state transition end. Inherited from State client.
  @return void
  **/
  void completeTransition(void);

};

} // namespace pandora_slam_2d


#endif  // PANDORA_SLAM_2D_PANDORA_SLAM_2D_H

