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
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

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
    void cannyThreshold();

    ros::NodeHandle node_handle_;
    ros::Publisher cloud_publisher_;

    message_filters::Subscriber<sensor_msgs::Image> *depth_image_subscriber_ptr_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_subscriber_ptr_;
    message_filters::Synchronizer<SyncPolicy> *synchronizer_ptr_;
    
    cv::Mat src, src_gray;
    cv::Mat dst, detected_edges;

    int edgeThresh;
    int lowThreshold;
    int max_lowThreshold;
    int ratio;
    int kernel_size;
    char* window_name;
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
    
    edgeThresh = 1;
    lowThreshold = 70;
    max_lowThreshold = 100;
    ratio = 3;
    kernel_size = 3;
    window_name = "Edge Map";
  }

  void PointCloudSubsampler::depthAndCloudCallback(
    const sensor_msgs::ImageConstPtr& depth_image,
    const sensor_msgs::PointCloud2ConstPtr& cloud_in)
  {
    //~ PointCloud pointCloud;
    //~ pcl::fromROSMsg(*cloud_in, pointCloud);
    //~ std::vector< int > index;
    //~ pcl::removeNaNFromPointCloud(pointCloud, pointCloud, index);
//~
    //~ // Correct dimensions published by gazebo plugin
    //~ pointCloud.width = 640;
    //~ pointCloud.height = 480;
    /// Convert sensor_msgs::Image to CvImage
    cv_bridge::CvImagePtr cv_depth_image_ptr =
      cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
    src = cv_depth_image_ptr->image;
    /// Create a matrix of the same type and size as src (for dst)
    dst.create(src.size(), src.type());
    /// Create a window
    //~ cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE );
    /// Show the image
    cannyThreshold();
  }

  void PointCloudSubsampler::cannyThreshold()
  {
    /// Reduce noise with a kernel 3x3
    cv::blur(src_gray, detected_edges, cv::Size(3,3));
    /// Canny detector
    cv::Canny(detected_edges, detected_edges, lowThreshold,
      lowThreshold * ratio, kernel_size );
    /// Using Canny's output as a mask, we display our result
    dst = cv::Scalar::all(0);

    src.copyTo(dst, detected_edges);
    cv::imshow(window_name, dst);
    cv::waitKey(1);
   }
}  // namespace pandora_slam

int main (int argc, char **argv)
{
  ros::init(argc,argv,"point_cloud_subsampler");
  pandora_slam::PointCloudSubsampler point_cloud_subsampler;

  ros::spin();
  return 0;
}
