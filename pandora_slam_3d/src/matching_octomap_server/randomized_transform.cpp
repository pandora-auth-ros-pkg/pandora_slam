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
#include "matching_octomap_server/randomized_transform.h"
namespace pandora_slam
{
  RandomizedTransform::RandomizedTransform()
    : initial_tf_(tf::Transform::getIdentity())
  {
    transform = initial_tf_;
    translation_range_ = 0.6;
    rotation_range_ = 1.57;
  }

  RandomizedTransform::RandomizedTransform(const tf::Transform& initial_tf,
    const double& translation_range, const double& rotation_range)
    : initial_tf_(initial_tf)
  {
    transform = initial_tf_;
    translation_range_ = translation_range;
    rotation_range_ = rotation_range;
  }

  RandomizedTransform::~RandomizedTransform()
  {
  }

  void RandomizedTransform::randomize()
  {
    double x, y, z, roll, pitch, yaw;
    tf::Vector3 origin = initial_tf_.getOrigin();
    tf::Matrix3x3 basis = initial_tf_.getBasis();
    x = origin[0] + static_cast<double>(rand()) / RAND_MAX *
      translation_range_ - translation_range_ / 2;
    y = origin[1] + static_cast<double>(rand()) / RAND_MAX *
      translation_range_ - translation_range_ / 2;
    //~ z = origin[2] + static_cast<double>(rand()) / RAND_MAX *
      //~ translation_range_ - translation_range_ / 2;
    basis.getRPY(roll, pitch, yaw);
    z = origin[2];
    //~ roll = roll + static_cast<double>(rand()) / RAND_MAX *
      //~ rotation_range_ - rotation_range_ / 2;
    //~ pitch = pitch + static_cast<double>(rand()) / RAND_MAX *
      //~ rotation_range_ - rotation_range_ / 2;
    yaw = yaw + static_cast<double>(rand()) / RAND_MAX *
      rotation_range_ - rotation_range_ / 2;

    tf::Vector3 new_origin(x, y, z);
    tf::Matrix3x3 new_basis;
    new_basis.setRPY(roll, pitch, yaw);
    tf::Transform new_transform(new_basis, new_origin);
    transform = new_transform;
  }
}  // namespace pandora_slam
