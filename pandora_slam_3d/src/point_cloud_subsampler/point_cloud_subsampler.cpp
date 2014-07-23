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
    cloud_subscriber_ = node_handle_.subscribe(
      "/kinect/depth_registered/points", 1,
      &PointCloudSubsampler::pointCloudCallback, this);

    cloud_publisher_ = node_handle_.advertise<PointCloud>(
      "/kinect/depth_registered/points/subsampled", 5);

    inflation_kernel_size_ = 15;
    blur_kernel_size_ = 7;
    neighbours_threshold_ = 30;
    dense_voxel_size_ = 0.06;
    sparse_voxel_size_  = 0.12;
    sensor_cutoff_ = 3.0;
    curvature_threshold_ = 0.06;
    curvature_min_distance_threshold_ = 0.65;
    curvature_max_distance_threshold_ = 2.0;
    show_curvature_image_ = false;
    show_inflation_image_ = false;

    normal_max_depth_change_factor_ = 0.02;
    normal_smoothing_size_ = 10.0;

    dynamic_reconfigure::Server<subsamplerConfig>::CallbackType
      callback_type;
    callback_type = boost::bind(&PointCloudSubsampler::reconfigureCallback,
      this, _1, _2);
    reconfigure_server_.setCallback(callback_type);
  }

  void PointCloudSubsampler::pointCloudCallback(
    const PointCloud::ConstPtr& input_cloud_ptr)
  {
    Timer::start("pointCloudCallback", "", true);
    Timer::start("preprocessPointCloud", "pointCloudCallback", false);

    /// Create a curvature image from point cloud
    cv::Mat* curvature_image_ptr(new cv::Mat);
    curvature_image_ptr->create(
      input_cloud_ptr->height, input_cloud_ptr->width, CV_32F);
    estimateCurvature(input_cloud_ptr, curvature_image_ptr);

    ///Remove noise from point curvature image
    removeNoise(curvature_image_ptr);

    if (show_curvature_image_)
    {
      cv::imshow("Curvature", *curvature_image_ptr);
      cv::waitKey(1);
    }

    /// Inflate edges
    inflateEdges(curvature_image_ptr);

    Timer::tick("preprocessPointCloud");
    Timer::start("voxel_grid", "pointCloudCallback", false);

    ///Create subsampled cloud
    PointCloud::Ptr subsampled_cloud(new PointCloud);
    subsampleCloud(input_cloud_ptr, curvature_image_ptr, subsampled_cloud);
    cloud_publisher_.publish(*subsampled_cloud);

    Timer::tick("voxel_grid");
    Timer::tick("pointCloudCallback");
    Timer::printAllMeansTree();

    delete curvature_image_ptr;
  }

  void PointCloudSubsampler::estimateCurvature(
    const PointCloud::ConstPtr& input_cloud_ptr, cv::Mat* curvature_image_ptr)
  {
    ///Estimate point cloud normal and curvature
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

    ///Create curvature image using thresholds for curvature value, min
    ///and max point distance
    for (int ii = 0; ii < curvature_image_ptr->cols *
      curvature_image_ptr->rows; ii++)
    {
      float curvature, distance;
      curvature = normalsPtr->points[ii].curvature;
      distance = input_cloud_ptr->points[ii].getVector3fMap().norm();
      uint8_t* curvature_data;
      if (pcl::isFinite(normalsPtr->points[ii]) &&
        curvature >= curvature_threshold_ &&
        distance <= curvature_max_distance_threshold_ &&
        distance >= curvature_min_distance_threshold_)
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
  }

  void PointCloudSubsampler::removeNoise(cv::Mat* curvature_image_ptr)
  {
    ///Make input image binary using a threshold
    cv::threshold(*curvature_image_ptr, *curvature_image_ptr, 1, 255,
      cv::THRESH_BINARY);
    ///Create a blurred copy of the image
    cv::Mat blurred_curvature = curvature_image_ptr->clone();
    cv::blur(*curvature_image_ptr, blurred_curvature, cv::Size(
      blur_kernel_size_, blur_kernel_size_), cv::Point(-1, -1));
    ///Use blurred image to filter out points with few neighbours
    for (int ii = 0; ii < curvature_image_ptr->cols *
      curvature_image_ptr->rows; ii++)
    {
      float neighbours = static_cast<float>(blurred_curvature.data[ii])
        / static_cast<float>(255) * blur_kernel_size_ * blur_kernel_size_;
      if (neighbours < neighbours_threshold_ ||
        curvature_image_ptr->data[ii] != 255)
      {
        curvature_image_ptr->data[ii] = 0;
      }
      else
      {
        curvature_image_ptr->data[ii] = 255;
      }
    }
  }

  void PointCloudSubsampler::normalizeImage(cv::Mat* image_ptr)
  {
    ///Normalize image from 1 to 255 and convert to CV_8UC1 encoding
    double min, max;
    cv::minMaxIdx(*image_ptr, &min, &max);
    *image_ptr = (*image_ptr - min) * 255 / (max - min);
    *image_ptr = cv::abs(*image_ptr);
    image_ptr->convertTo(*image_ptr, CV_8UC1);
  }

  void PointCloudSubsampler::inflateEdges(cv::Mat* edges)
  {
    ///Inflate edges with a box filter and then convert it to binary
    cv::Mat dst;
    cv::boxFilter(*edges, *edges, edges->type(), cv::Size(
      inflation_kernel_size_, inflation_kernel_size_), cv::Point(-1, -1),
      false);

    cv::threshold(*edges, *edges, 1, 255, cv::THRESH_BINARY);

    if (show_inflation_image_)
    {
    dst.create(edges->size(), edges->type());
    dst = cv::Scalar::all(0);
    edges->copyTo(dst, *edges);
      cv::imshow("Inflated image", dst);
      cv::waitKey(1);
    }
  }

  void PointCloudSubsampler::voxelFilter(
    PointCloud::Ptr input_cloud_ptr, double voxel_size)
  {
    ///Use a voxel grid filter to subsample a point cloud
    pcl::PCLPointCloud2::Ptr cloud_ptr (new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filtered_ptr (new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*input_cloud_ptr, *cloud_ptr);
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
    voxel_grid.setInputCloud(cloud_ptr);
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_grid.filter(*cloud_filtered_ptr);
    pcl::fromPCLPointCloud2(*cloud_filtered_ptr, *input_cloud_ptr);
  }

  void PointCloudSubsampler::subsampleCloud(
    const PointCloud::ConstPtr& input_cloud_ptr, cv::Mat* curvature_image_ptr,
    PointCloud::Ptr subsampled_cloud)
  {
    /// Create edge and non-edge point cloud
    PointCloud::Ptr edge_cloud_ptr(new PointCloud);
    PointCloud::Ptr non_edge_cloud_ptr(new PointCloud);
    for (int ii = 0; ii < curvature_image_ptr->cols *
      curvature_image_ptr->rows; ii++)
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
    voxelFilter(edge_cloud_ptr, dense_voxel_size_);
    voxelFilter(non_edge_cloud_ptr, sparse_voxel_size_);

    *subsampled_cloud = *edge_cloud_ptr + *non_edge_cloud_ptr;
    subsampled_cloud->header = input_cloud_ptr->header;
  }

  void PointCloudSubsampler::reconfigureCallback(
    pandora_slam_3d::point_cloud_subsamplerConfig &config, uint32_t level)
  {
    inflation_kernel_size_ = 1 + 2 * config.inflation_size;
    blur_kernel_size_ = 1 + 2 * config.blur_size;
    neighbours_threshold_ = config.neighbours_threshold;
    dense_voxel_size_ = config.dense_voxel_size;
    sparse_voxel_size_ = config.sparse_voxel_size;
    sensor_cutoff_ = config.sensor_cutoff;

    curvature_threshold_ = config.curvature_threshold;
    curvature_min_distance_threshold_ = config.curvature_min_distance_threshold;
    curvature_max_distance_threshold_ = config.curvature_max_distance_threshold;
    show_curvature_image_ = config.show_curvature_image;
    show_inflation_image_ = config.show_inflation_image;

    normal_max_depth_change_factor_ = config.normal_max_depth_change_factor;
    normal_smoothing_size_ = config.normal_smoothing_size;
  }
}  // namespace pandora_slam

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_subsampler");
  pandora_slam::PointCloudSubsampler point_cloud_subsampler;

  ros::spin();
  return 0;
}
