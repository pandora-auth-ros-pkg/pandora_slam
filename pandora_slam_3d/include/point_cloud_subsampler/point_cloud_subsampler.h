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
#ifndef POINT_CLOUD_SUBSAMPLER_POINT_CLOUD_SUBSAMPLER_H
#define POINT_CLOUD_SUBSAMPLER_POINT_CLOUD_SUBSAMPLER_H

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <pandora_slam_3d/point_cloud_subsamplerConfig.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include "utils/timer.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pandora_slam_3d::point_cloud_subsamplerConfig subsamplerConfig;

namespace pandora_slam
{
  class PointCloudSubsampler
  {
   public:
    PointCloudSubsampler();
   private:
    void pointCloudCallback(const PointCloud::ConstPtr& input_cloud_ptr);
    void estimateCurvature(const PointCloud::ConstPtr& input_cloud_ptr,
      cv::Mat* curvature_image_ptr);
    void removeNoise(cv::Mat* curvature_image_ptr);
    void normalizeImage(cv::Mat* image_ptr);
    void inflateEdges(cv::Mat* edges);
    void voxelFilter(PointCloud::Ptr input_cloud_ptr, double voxel_size);
    void subsampleCloud(const PointCloud::ConstPtr& input_cloud_ptr,
      cv::Mat* curvature_image_ptr, PointCloud::Ptr subsampled_cloud);
    void reconfigureCallback(
      pandora_slam_3d::point_cloud_subsamplerConfig &config,
      uint32_t level);

    ros::NodeHandle node_handle_;
    ros::Publisher cloud_publisher_;
    ros::Subscriber cloud_subscriber_;

    int inflation_kernel_size_;
    int blur_kernel_size_;
    int neighbours_threshold_;
    double dense_voxel_size_;
    double sparse_voxel_size_;
    double sensor_cutoff_;
    double curvature_threshold_;
    double curvature_min_distance_threshold_;
    double curvature_max_distance_threshold_;
    bool detect_edges_;
    bool show_curvature_image_;
    bool show_inflation_image_;
    double normal_max_depth_change_factor_;
    double normal_smoothing_size_;
    dynamic_reconfigure::Server<pandora_slam_3d::point_cloud_subsamplerConfig>
      reconfigure_server_;
  };
}  // namespace pandora_slam
#endif  // POINT_CLOUD_SUBSAMPLER_POINT_CLOUD_SUBSAMPLER_H
