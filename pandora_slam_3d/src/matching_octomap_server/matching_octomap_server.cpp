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
  }

  MatchingOctomapServer::~MatchingOctomapServer()
  {
  }

  void MatchingOctomapServer::matchCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud,
    const sensor_msgs::PointCloud2::ConstPtr& subsampled_cloud)
  {
    ros::WallTime startTime = ros::WallTime::now();

    // ground filtering in base frame
    PCLPointCloud pc; // input cloud for filtering and ground-detection
    pcl::fromROSMsg(*cloud, pc);

    tf::StampedTransform sensorToWorldTf;
    try
    {
      m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id,
        cloud->header.stamp, sensorToWorldTf);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() <<
        ",quitting callback");
      return;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

    // set up filter for height range, also removes NANs:
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

    PCLPointCloud pc_ground; // segmented ground plane
    PCLPointCloud pc_nonground; // everything else

    if (m_filterGroundPlane)
    {
      tf::StampedTransform sensorToBaseTf, baseToWorldTf;
      try
      {
        m_tfListener.waitForTransform(m_baseFrameId, cloud->header.frame_id,
          cloud->header.stamp, ros::Duration(0.2));
        m_tfListener.lookupTransform(m_baseFrameId, cloud->header.frame_id,
          cloud->header.stamp, sensorToBaseTf);
        m_tfListener.lookupTransform(m_worldFrameId, m_baseFrameId,
          cloud->header.stamp, baseToWorldTf);
      }
      catch(tf::TransformException& ex)
      {
        ROS_ERROR_STREAM("Transform error for ground plane filter: " <<
          ex.what() << ", quitting callback.\n" << "You need to set " <<
          "the base_frame_id or disable filter_ground.");
      }

      Eigen::Matrix4f sensorToBase, baseToWorld;
      pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);
      pcl_ros::transformAsMatrix(baseToWorldTf, baseToWorld);

      // transform pointcloud from sensor frame to fixed robot frame
      pcl::transformPointCloud(pc, pc, sensorToBase);
      pass.setInputCloud(pc.makeShared());
      pass.filter(pc);
      filterGroundPlane(pc, pc_ground, pc_nonground);

      // transform clouds to world frame for insertion
      pcl::transformPointCloud(pc_ground, pc_ground, baseToWorld);
      pcl::transformPointCloud(pc_nonground, pc_nonground, baseToWorld);
    }
    else
    {
      // directly transform to map frame:
      pcl::transformPointCloud(pc, pc, sensorToWorld);

      // just filter height range:
      pass.setInputCloud(pc.makeShared());
      pass.filter(pc);

      pc_nonground = pc;
      // pc_nonground is empty without ground segmentation
      pc_ground.header = pc.header;
      pc_nonground.header = pc.header;
    }
    insertScan(sensorToWorldTf.getOrigin(), pc_ground, pc_nonground);

    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_DEBUG_STREAM("Pointcloud insertion in OctomapServer done (" <<
      pc_ground.size() << "+" << pc_nonground.size() << " pts " <<
      "(ground/nonground), " << total_elapsed <<" sec)");
    publishAll(cloud->header.stamp);
  }
}  // namespace pandora_slam

int main(int argc, char **argv)
{
  ros::init(argc, argv, "matching_octomap_server");
  pandora_slam::MatchingOctomapServer matching_octomap_server;

  ros::spin();
  return 0;
}
