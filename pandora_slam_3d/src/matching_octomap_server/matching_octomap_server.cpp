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
#include "matching_octomap_server/matching_octomap_server.h"
namespace pandora_slam
{
  MatchingOctomapServer::MatchingOctomapServer()
  {
    subsampled_cloud_subscriber_ = m_nh.subscribe(
      "/kinect/depth_registered/points", 1,
      &MatchingOctomapServer::matchCloudCallback, this);
  }

  MatchingOctomapServer::~MatchingOctomapServer()
  {
  }

  void MatchingOctomapServer::matchCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& subsampled_cloud)
  {
    Timer::start("matchCloudCallback", "", true);
    Timer::start("computeFitness", "matchCloudCallback", false);

    PCLPointCloud subsampled_pc; // input cloud for matching
    pcl::fromROSMsg(*subsampled_cloud, subsampled_pc);

    tf::StampedTransform sensorToWorldTf;
    try
    {
      m_tfListener.lookupTransform(m_worldFrameId,
        subsampled_cloud->header.frame_id,
        subsampled_cloud->header.stamp, sensorToWorldTf);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() <<
        ",quitting callback");
      return;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
    pcl::transformPointCloud(subsampled_pc, subsampled_pc, sensorToWorld);

    int points_size = subsampled_pc.width * subsampled_pc.height;
    double fitness = 0;
    for (int ii = 0; ii < points_size; ii++)
    {
      if (pcl::isFinite(subsampled_pc.points[ii]))
      {
        octomap::OcTreeNode* node_ptr;
        node_ptr = m_octree->search(subsampled_pc.points[ii].x,
          subsampled_pc.points[ii].y, subsampled_pc.points[ii].z);
        if (node_ptr != NULL)
        {
          fitness += node_ptr->getOccupancy();
        }
      }
    }
    fitness = fitness / points_size;
    ROS_INFO_STREAM("Fitness: " << fitness);

    Timer::tick("computeFitness");
    Timer::tick("matchCloudCallback");
    Timer::printAllMeansTree();
  }
}  // namespace pandora_slam

int main(int argc, char **argv)
{
  ros::init(argc, argv, "matching_octomap_server");
  pandora_slam::MatchingOctomapServer matching_octomap_server;

  ros::spin();
  return 0;
}
