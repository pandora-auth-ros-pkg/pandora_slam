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
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_saver");

  ros::NodeHandle node_handle;
  tf::TransformListener tf_listener;
  ros::Time previous_stamp;
  
  std::string folder_name;
  node_handle.getParam("/trajectory_saver/folder_name", folder_name);

  std::ofstream file_;
  char buffer[20]; 
  int n;
  n = sprintf(buffer, "%u", ros::WallTime::now().sec);
  std::string name = buffer;
  name = "/home/vagelis/trajectories/" + folder_name + "/trajectory_" + name + ".txt";
  file_.open(name.c_str());
  file_ << "# ground truth trajectory\n";
  file_ << "# timestamp tx ty tz qx qy qz qw\n";

  ros::Rate r(100);
  while (ros::ok())
  {
    tf::StampedTransform trajectory_tf;
    try{
      tf_listener.lookupTransform("world_old", "openni_rgb_optical_frame", 
        ros::Time(0), trajectory_tf);
    }
    catch (tf::TransformException ex){
      continue;
    }
    if (previous_stamp != trajectory_tf.stamp_)
    {
      file_ << trajectory_tf.stamp_ << " ";
      file_ << trajectory_tf.getOrigin()[0] << " ";
      file_ << trajectory_tf.getOrigin()[1] << " ";
      file_ << trajectory_tf.getOrigin()[2] << " ";
      file_ << trajectory_tf.getRotation().getAxis()[0] << " ";
      file_ << trajectory_tf.getRotation().getAxis()[1] << " ";
      file_ << trajectory_tf.getRotation().getAxis()[2] << " ";
      file_ << trajectory_tf.getRotation().getW() << "\n";
      previous_stamp = trajectory_tf.stamp_;
    }
    r.sleep();
  }
  file_.close();
  return 0;
}
