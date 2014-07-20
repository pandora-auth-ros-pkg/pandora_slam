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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <cv_bridge/cv_bridge.h>
#include "point_cloud_subsampler/edge_detector.h"
#include <dynamic_reconfigure/server.h>
#include <pandora_slam_3d/point_cloud_subsamplerConfig.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/point_tests.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::Image, sensor_msgs::PointCloud2> SyncPolicy;

namespace pandora_slam
{
  class PointCloudSubsampler
  {
   public:
    PointCloudSubsampler();
   private:
    void depthAndCloudCallback(
      const sensor_msgs::ImageConstPtr& depth_image,
      const sensor_msgs::PointCloud2ConstPtr& cloud_in);
    //~ cv::Mat preprocessDepth(const sensor_msgs::ImageConstPtr& depth_image);
    void preprocessPointCloud(PointCloud::Ptr input_cloud_ptr,
      boost::shared_ptr<cv::Mat> curvature_image);
    void normalizeImage(boost::shared_ptr<cv::Mat> image);
    void reconfigureCallback(pandora_slam_3d::point_cloud_subsamplerConfig &config,
      uint32_t level);

    ros::NodeHandle node_handle_;
    ros::Publisher cloud_publisher_;

    message_filters::Subscriber<sensor_msgs::Image> *depth_image_subscriber_ptr_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_subscriber_ptr_;
    message_filters::Synchronizer<SyncPolicy> *synchronizer_ptr_;
    
    EdgeDetector edge_detector_;
    int edge_detection_method_;
    int inflation_size_;
    double curvature_threshold_;
    double curvature_distance_threshold_;
    bool show_curvature_image_;
    double normal_max_depth_change_factor_;
    double normal_smoothing_size_;
    dynamic_reconfigure::Server<pandora_slam_3d::point_cloud_subsamplerConfig>
      reconfigure_server_;
  };

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
      &PointCloudSubsampler::depthAndCloudCallback,this, _1, _2));

    cloud_publisher_ = node_handle_.advertise<PointCloud>(
      "/kinect/depth_registered/points/subsampled", 5);

    edge_detection_method_ = EdgeDetector::CANNY;
    inflation_size_ = 15;
    curvature_threshold_ = 0.7;
    curvature_distance_threshold_ = 0.7;
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
    /// Create pcl Point Cloud from sensor_msgs::PointCloud2
    PointCloud::Ptr input_cloud_ptr(new PointCloud);
    pcl::fromROSMsg(*cloud_in, *input_cloud_ptr);

    //~ cv::Mat edges = preprocessDepth(depth_image);
    //~ cv::Mat edges = preprocessPointCloud(input_cloud_ptr);
    boost::shared_ptr<cv::Mat> curvature_image(new cv::Mat);
    curvature_image->create(
      input_cloud_ptr->height, input_cloud_ptr->width, CV_32F);
    preprocessPointCloud(input_cloud_ptr, curvature_image);

    //~ /// Create edge point cloud
    //~ PointCloud edgePointCloud;
    //~ PointCloud nonEdgePointCloud;
    //~ for (int ii = 0; ii < edges.cols * edges.rows; ii++)
    //~ {
      //~ if (pcl::isFinite(input_cloud_ptr->at(ii)))
      //~ {
        //~ if (edges.data[ii] == 255)
        //~ {
          //~ edgePointCloud.push_back(input_cloud_ptr->at(ii));
        //~ }
        //~ else
        //~ {
          //~ nonEdgePointCloud.push_back(input_cloud_ptr->at(ii));
        //~ }
      //~ }
    //~ }
    //~ edgePointCloud.header = input_cloud_ptr->header;
    //~ nonEdgePointCloud.header = input_cloud_ptr->header;
    //~ cloud_publisher_.publish(edgePointCloud);
    //~ cloud_publisher_.publish(nonEdgePointCloud);
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
    //~ edge_detector_.inflateEdges(edges, inflation_size_);
    //~ return edges;
  //~ }

  void PointCloudSubsampler::preprocessPointCloud(
    PointCloud::Ptr input_cloud_ptr,
    boost::shared_ptr<cv::Mat> curvature_image)
  {
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

    for (int ii = 0; ii < curvature_image->cols * curvature_image->rows; ii++)
    {
      uint8_t* curvature_data;
      if (pcl::isFinite(normalsPtr->points[ii]) &&
        normalsPtr->points[ii].curvature >= curvature_threshold_)
      {
        //~ Eigen::Vector3f point;
        //~ point[0] = input_cloud_ptr->points[ii].data[0];
        //~ point[1] = input_cloud_ptr->points[ii].data[1];
        //~ point[2] = input_cloud_ptr->points[ii].data[2];
        //~ ROS_ERROR_STREAM(point.norm());
        curvature_data = reinterpret_cast<uint8_t*>(
          &(normalsPtr->points[ii].curvature));
        for (int jj = 0; jj < 4; jj++)
        {
          curvature_image->data[ii * 4 + jj] = curvature_data[jj];
        }
      }
      else
      {
        for (int jj = 0; jj < 4; jj++)
        {
          curvature_image->data[ii * 4 + jj] = 0;
        }
      }

    }
    normalizeImage(curvature_image);

    if (show_curvature_image_)
    {
      cv::imshow("Curvature", *curvature_image);
      cv::waitKey(1);
    }

    /// Detect edges
    //~ cv::Mat edges;
    //~ cv::Mat edges = edge_detector_.detect(
      //~ normalized_curvature_image, edge_detection_method_);
    //~ /// Inflate edges
    //~ edge_detector_.inflateEdges(edges, inflation_size_);
    //~ return normalized_curvature_image;
  }

  void PointCloudSubsampler::normalizeImage(boost::shared_ptr<cv::Mat> image)
  {
    double min, max;
    cv::minMaxIdx(*image, &min, &max);
    *image = (*image - min) * 255 / (max - min);
    *image = cv::abs(*image);
    image->convertTo(*image, CV_8UC1);
  }

  void PointCloudSubsampler::reconfigureCallback(
    pandora_slam_3d::point_cloud_subsamplerConfig &config, uint32_t level)
  {
    inflation_size_ = config.inflation_size;
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

int main (int argc, char **argv)
{
  ros::init(argc,argv,"point_cloud_subsampler");
  pandora_slam::PointCloudSubsampler point_cloud_subsampler;

  ros::spin();
  return 0;
}
