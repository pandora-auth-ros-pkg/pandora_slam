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
    point_cloud_subscriber_ =
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(
      m_nh, "/kinect/depth_registered/points", 1);
    subsampled_cloud_subscriber_=
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(
      m_nh, "/kinect/depth_registered/points/subsampled", 1);

    synchronizer_ = new message_filters::Synchronizer<PCSyncPolicy>(
      PCSyncPolicy(10), *point_cloud_subscriber_,
      *subsampled_cloud_subscriber_);
    synchronizer_->registerCallback(boost::bind(
      &MatchingOctomapServer::matchCloudCallback, this, _1, _2));

    cloud_publisher_ = m_nh.advertise<sensor_msgs::PointCloud2>(
      "/kinect/depth_registered/points/matched", 5);

    previous_tf_ = tf::Transform::getIdentity();
    tf_broadcaster_.sendTransform(tf::StampedTransform(previous_tf_,
      ros::Time::now(), m_worldFrameId, m_baseFrameId));
  }

  MatchingOctomapServer::~MatchingOctomapServer()
  {
  }

  void MatchingOctomapServer::matchCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& full_cloud,
    const sensor_msgs::PointCloud2::ConstPtr& subsampled_cloud)
  {
    ///Check if octree is initialize
    if (m_octree->getRoot() == NULL)
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(previous_tf_,
        ros::Time::now(), m_worldFrameId, m_baseFrameId));
      cloud_publisher_.publish(full_cloud);
      return;
    }
    Timer::start("matchCloudCallback", "", true);
    Timer::start("computeFitness", "matchCloudCallback", false);

    PCLPointCloud subsampled_pc; // input cloud for matching
    pcl::fromROSMsg(*subsampled_cloud, subsampled_pc);

    ///Transform cloud to m_baseFrameId
    tf::StampedTransform sensorToBaseTf;
    try
    {
      m_tfListener.lookupTransform(m_baseFrameId,
        subsampled_cloud->header.frame_id,
        subsampled_cloud->header.stamp, sensorToBaseTf);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() <<
        ",quitting callback");
      return;
    }

    Eigen::Matrix4f sensorToBase;
    pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);
    pcl::transformPointCloud(subsampled_pc, subsampled_pc, sensorToBase);

    ///Search for movement transform with RRHC
    RandomizedTransform random_transform(previous_tf_, 0.2, 0.5);
    tf::Transform best_transform = previous_tf_;
    double best_fitness = 0;
    double fitness;
    PCLPointCloud cloud;
    Eigen::Matrix4f baseToWorld;
    int points_size;
    for (int kk = 0; kk < 1000; kk++)
    {
      fitness = 0;

      pcl_ros::transformAsMatrix(random_transform.transform, baseToWorld);
      pcl::transformPointCloud(subsampled_pc, cloud, baseToWorld);

      points_size = cloud.width * cloud.height;

      octomap::OcTreeKey key;
      octomap::OcTreeNode* node_ptr;
      for (int ii = 0; ii < points_size; ii++)
      {
        if (m_octree->coordToKeyChecked(cloud.points[ii].x,
          cloud.points[ii].y, cloud.points[ii].z, key))
        {
          node_ptr = m_octree->search(cloud.points[ii].x, cloud.points[ii].y,
            cloud.points[ii].z);
          if (node_ptr == NULL)
          {
            ///Occupancy for unknown cells is considered 0.5
            fitness += 0.5;
          }
          else{
            fitness += node_ptr->getOccupancy();
          }
        }
      }
      fitness = fitness / points_size;
      if (fitness > best_fitness)
      {
        best_transform = random_transform.transform;
        best_fitness = fitness;
      }
      random_transform.randomize();
    }

    ///Print results
    tf::Vector3 new_origin = best_transform.getOrigin();
    tf::Matrix3x3 new_basis = best_transform.getBasis();
    double roll, pitch, yaw;
    new_basis.getRPY(roll, pitch, yaw);

    ROS_INFO_STREAM("Best fitness: " << best_fitness);
    ROS_INFO_STREAM("New transform:");
    ROS_INFO_STREAM("x: " << new_origin[0]);
    ROS_INFO_STREAM("y: " << new_origin[1]);
    ROS_INFO_STREAM("z: " << new_origin[2]);
    ROS_INFO_STREAM("R: " << roll / 3.14 * 180);
    ROS_INFO_STREAM("P: " << pitch / 3.14 * 180);
    ROS_INFO_STREAM("Y: " << yaw / 3.14 * 180);
    
    ///Broadcast new tf
    tf_broadcaster_.sendTransform(tf::StampedTransform(best_transform,
      subsampled_cloud->header.stamp, m_worldFrameId, m_baseFrameId));
    previous_tf_ = best_transform;

    Timer::tick("computeFitness");
    Timer::tick("matchCloudCallback");
    Timer::printAllMeansTree();

    cloud_publisher_.publish(subsampled_cloud);
  }
}  // namespace pandora_slam

int main(int argc, char **argv)
{
  ros::init(argc, argv, "matching_octomap_server");
  pandora_slam::MatchingOctomapServer matching_octomap_server;

  ros::spin();
  return 0;
}
