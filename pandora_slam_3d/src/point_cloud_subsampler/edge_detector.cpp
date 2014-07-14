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
#include "point_cloud_subsampler/edge_detector.h"

namespace pandora_slam
{
  EdgeDetector::EdgeDetector()
  {
    canny_low_threshold_ = 10;
    canny_ratio_ = 3;
    canny_kernel_size_ = 3;
    
    scharr_scale_ = 1;
    scharr_delta_ = 0;
    scharr_ddepth_ = CV_16S;

    show_edges_image_ = false;
    show_inflation_image_ = false;
  }

  cv::Mat EdgeDetector::detect(const cv::Mat &src, int method)
  {
    if (method == CANNY)
    {
      return cannyEdges(src);
    }
    else if (method == SCHARR)
    {
      return scharrDerivatives(src);
    }
    else if (method == CURVATURE)
    {
      return cannyEdges(scharrDerivatives(src));
    }
    else
    {
      ROS_ERROR_STREAM("Computation method passed doesn't exist. " <<
        "Returning black image");
      cv::Mat dst;
      dst.create(src.size(), src.type());
      dst = cv::Scalar::all(0);
      return dst;
    }
   }

  cv::Mat EdgeDetector::cannyEdges(const cv::Mat &src)
  {
    cv::Mat dst, detected_edges;

    /// Reduce noise with a kernel 3x3
    cv::blur(src, detected_edges, cv::Size(3,3));
    if (show_edges_image_)
    {
      cv::imshow("Source Map", detected_edges);
      cv::waitKey(1);
    }
    /// Canny detector
    cv::Canny(detected_edges, detected_edges, canny_low_threshold_,
      canny_low_threshold_ * canny_ratio_, canny_kernel_size_ );
    /// Create a matrix of the same type and size as src (for dst)
    dst.create(src.size(), src.type());
    /// Using Canny's output as a mask, we display our result
    dst = cv::Scalar::all(0);
    src.copyTo(dst, detected_edges);

    if (show_edges_image_)
    {
      cv::imshow("Edge Map", dst);
      cv::waitKey(1);
    }
    
    return detected_edges;
  }

  cv::Mat EdgeDetector::scharrDerivatives(const cv::Mat &src)
  {
    cv::Mat src_blurred;
    cv::Mat grad;
    cv::GaussianBlur(src, src_blurred, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);

    /// Generate grad_x and grad_y
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    cv::Scharr(src_blurred, grad_x, scharr_ddepth_, 1, 0, scharr_scale_,
      scharr_delta_, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);

    /// Gradient Y
    cv::Scharr(src_blurred, grad_y, scharr_ddepth_, 0, 1, scharr_scale_,
      scharr_delta_, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    /// Total Gradient (approximate)
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

    if (show_edges_image_)
    {
      cv::imshow("Source Map", src_blurred);
      cv::imshow("Scharr", grad );
      cv::imshow("abs_grad_x", abs_grad_x );
      cv::imshow("abs_grad_y", abs_grad_y );
      cv::waitKey(1);
    }

    return grad;
  }
  
  void EdgeDetector::inflateEdges(cv::Mat &edges, int inflation_size)
  {
    if (inflation_size < 3)
    {
      inflation_size = 3;
    }
    else if (inflation_size % 2 != 1)
    {
      inflation_size++;
    }
    cv::Mat dst;
    cv::boxFilter(
      edges, edges, edges.type(), cv::Size(inflation_size, inflation_size),
      cv::Point(-1, -1), false);

    for (int ii = 0; ii < edges.cols * edges.rows; ii++)
    {
      if (edges.data[ii] != 0)
      {
        edges.data[ii] == 255;
      }
    }

    dst.create(edges.size(), edges.type());
    dst = cv::Scalar::all(0);
    edges.copyTo(dst, edges);
    if (show_inflation_image_)
    {
      cv::imshow("Inflated image", dst);
      cv::waitKey(1);
    }
  }
}  // namespace pandora_slam
