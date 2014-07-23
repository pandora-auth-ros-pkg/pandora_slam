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
#include "point_cloud_subsampler/point_cloud_subsampler.h"

namespace pandora_slam
{
  PointCloudSubsampler::PointCloudSubsampler()
  {
    depth_image_subscriber_ptr_ =
      new message_filters::Subscriber<sensor_msgs::Image>(
        node_handle_, "/kinect/depth_registered/image_raw", 1);
    cloud_subscriber_ptr_ =
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(
        node_handle_, "/kinect/depth_registered/points", 1);
    synchronizer_ptr_ =
      new message_filters::Synchronizer<SyncPolicy>(
        SyncPolicy(5), *depth_image_subscriber_ptr_, *cloud_subscriber_ptr_);
    synchronizer_ptr_->registerCallback(boost::bind(
      &PointCloudSubsampler::depthAndCloudCallback, this, _1, _2));

    cloud_publisher_ = node_handle_.advertise<PointCloud>(
      "/kinect/depth_registered/points/subsampled", 5);

    edge_detection_method_ = EdgeDetector::CANNY;
    inflation_kernel_size_ = 15;
    dense_voxel_size_ = 0.06;
    sparse_voxel_size_  = 0.12;
    sensor_cutoff_ = 3.0;
    curvature_threshold_ = 0.06;
    curvature_distance_threshold_ = 2.0;
    show_curvature_image_ = false;

    normal_max_depth_change_factor_ = 0.02;
    normal_smoothing_size_ = 10.0;

    dynamic_reconfigure::Server<pandora_slam_3d::point_cloud_subsamplerConfig>::CallbackType
      callback_type;
    callback_type = boost::bind(&PointCloudSubsampler::reconfigureCallback, this, _1, _2);
    reconfigure_server_.setCallback(callback_type);
  }

  void PointCloudSubsampler::depthAndCloudCallback(
    const sensor_msgs::ImageConstPtr& depth_image,
    const sensor_msgs::PointCloud2ConstPtr& cloud_in)
  {
    Timer::start("depthAndCloudCallback", "", true);
    Timer::start("fromROSMsg", "depthAndCloudCallback", false);
    /// Create pcl Point Cloud from sensor_msgs::PointCloud2
    PointCloud::Ptr input_cloud_ptr(new PointCloud);
    pcl::fromROSMsg(*cloud_in, *input_cloud_ptr);

    Timer::tick("fromROSMsg");

    boost::shared_ptr<cv::Mat> curvature_image_ptr(new cv::Mat);
    curvature_image_ptr->create(
      input_cloud_ptr->height, input_cloud_ptr->width, CV_32F);
    preprocessPointCloud(input_cloud_ptr, curvature_image_ptr);

    Timer::start("voxel_grid", "depthAndCloudCallback", false);
    /// Create edge point cloud
    PointCloud::Ptr edge_cloud_ptr(new PointCloud);
    PointCloud::Ptr non_edge_cloud_ptr(new PointCloud);
    for (int ii = 0; ii < curvature_image_ptr->cols * curvature_image_ptr->rows; ii++)
    {
      if (pcl::isFinite(input_cloud_ptr->at(ii)) &&
        input_cloud_ptr->points[ii].getVector3fMap().norm() <= sensor_cutoff_)
      {
        if (curvature_image_ptr->data[ii] == 255)
        {
          edge_cloud_ptr->push_back(input_cloud_ptr->at(ii));
        }
        else
        {
          non_edge_cloud_ptr->push_back(input_cloud_ptr->at(ii));
        }
      }
    }

    edge_cloud_ptr->header = input_cloud_ptr->header;
    non_edge_cloud_ptr->header = input_cloud_ptr->header;
    subsampleCloud(edge_cloud_ptr, dense_voxel_size_);
    subsampleCloud(non_edge_cloud_ptr, sparse_voxel_size_);

    PointCloud subsampled_cloud;
    subsampled_cloud = *edge_cloud_ptr + *non_edge_cloud_ptr;
    subsampled_cloud.header = input_cloud_ptr->header;
    cloud_publisher_.publish(subsampled_cloud);
    Timer::tick("voxel_grid");
    Timer::tick("depthAndCloudCallback");
    Timer::printAllMeansTree();
  }

  //~ cv::Mat PointCloudSubsampler::preprocessDepth(
    //~ const sensor_msgs::ImageConstPtr& depth_image)
  //~ {
    //~ /// Convert sensor_msgs::Image to CvImage
    //~ cv_bridge::CvImageConstPtr cv_depth_image_ptr =
      //~ cv_bridge::toCvShare(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
//~
    //~ ///Scale image and change encoding
    //~ cv::Mat image = cv_depth_image_ptr->image;
    //~ normalizeImage(image);;
//~
    //~ /// Detect edges
    //~ cv::Mat edges = edge_detector_.detect(image, edge_detection_method_);
    //~ /// Inflate edges
    //~ edge_detector_.inflateEdges(edges, inflation_kernel_size_);
    //~ return edges;
  //~ }

  void PointCloudSubsampler::preprocessPointCloud(
    PointCloud::Ptr input_cloud_ptr,
    boost::shared_ptr<cv::Mat> curvature_image_ptr)
  {
    Timer::start("preprocessPointCloud", "depthAndCloudCallback", false);
    Timer::start("IntegralImageNormalEstimation", "preprocessPointCloud", false);
    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr
      normalsPtr(new pcl::PointCloud<pcl::Normal>);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>
      normalEstimator;
    normalEstimator.setNormalEstimationMethod(
      normalEstimator.COVARIANCE_MATRIX);
    normalEstimator.setMaxDepthChangeFactor(normal_max_depth_change_factor_);
    normalEstimator.setNormalSmoothingSize(normal_smoothing_size_);
    normalEstimator.setInputCloud(input_cloud_ptr);
    normalEstimator.compute(*normalsPtr);

    for (int ii = 0; ii < curvature_image_ptr->cols * curvature_image_ptr->rows; ii++)
    {
      float curvature, distance;
      curvature = normalsPtr->points[ii].curvature;
      distance = input_cloud_ptr->points[ii].getVector3fMap().norm();
      uint8_t* curvature_data;
      if (pcl::isFinite(normalsPtr->points[ii]) &&
        curvature >= curvature_threshold_ &&
        distance <= curvature_distance_threshold_)
      {
        curvature_data = reinterpret_cast<uint8_t*>(
          &(normalsPtr->points[ii].curvature));
        for (int jj = 0; jj < 4; jj++)
        {
          curvature_image_ptr->data[ii * 4 + jj] = curvature_data[jj];
        }
      }
      else
      {
        for (int jj = 0; jj < 4; jj++)
        {
          curvature_image_ptr->data[ii * 4 + jj] = 0;
        }
      }
    }

    normalizeImage(curvature_image_ptr);
    if (show_curvature_image_)
    {
      cv::imshow("Curvature", *curvature_image_ptr);
      cv::waitKey(1);
    }

    Timer::tick("IntegralImageNormalEstimation");
    Timer::start("inflation", "preprocessPointCloud", false);
    /// Inflate edges
    edge_detector_.inflateEdges(curvature_image_ptr, inflation_kernel_size_);
    Timer::tick("inflation");
    Timer::tick("preprocessPointCloud");
  }

  void PointCloudSubsampler::normalizeImage(
    boost::shared_ptr<cv::Mat> image_ptr)
  {
    double min, max;
    cv::minMaxIdx(*image_ptr, &min, &max);
    *image_ptr = (*image_ptr - min) * 255 / (max - min);
    *image_ptr = cv::abs(*image_ptr);
    image_ptr->convertTo(*image_ptr, CV_8UC1);
  }

  void PointCloudSubsampler::subsampleCloud(
    PointCloud::Ptr input_cloud_ptr, double voxel_size)
  {
    pcl::PCLPointCloud2::Ptr cloud_ptr (new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filtered_ptr (new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*input_cloud_ptr, *cloud_ptr);
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
    voxel_grid.setInputCloud(cloud_ptr);
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_grid.filter(*cloud_filtered_ptr);
    pcl::fromPCLPointCloud2(*cloud_filtered_ptr, *input_cloud_ptr);
  }

  void PointCloudSubsampler::reconfigureCallback(
    pandora_slam_3d::point_cloud_subsamplerConfig &config, uint32_t level)
  {
    inflation_kernel_size_ = 1 + 2 * config.inflation_size;
    dense_voxel_size_ = config.dense_voxel_size;
    sparse_voxel_size_ = config.sparse_voxel_size;
    sensor_cutoff_ = config.sensor_cutoff;

    curvature_threshold_ = config.curvature_threshold;
    curvature_distance_threshold_ = config.curvature_distance_threshold;
    show_curvature_image_ = config.show_curvature_image;

    edge_detection_method_ = config.edge_detection_method;
    normal_max_depth_change_factor_ = config.normal_max_depth_change_factor;
    normal_smoothing_size_ = config.normal_smoothing_size;

    edge_detector_.setCannyLowThreshold(config.canny_low_threshold);
    edge_detector_.setCannyHighThreshold(config.canny_high_threshold);

    edge_detector_.setScharrScale(config.scharr_scale);
    edge_detector_.setScharrDelta(config.scharr_delta);

    edge_detector_.setShowEdgesImage(config.show_edges_image);
    edge_detector_.setShowInflationImage(config.show_inflation_image);
  }
}  // namespace pandora_slam

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_subsampler");
  pandora_slam::PointCloudSubsampler point_cloud_subsampler;

  ros::spin();
  return 0;
}
