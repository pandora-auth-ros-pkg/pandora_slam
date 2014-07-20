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
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace pandora_slam
{
  class EdgeDetector
  {
   public:
    enum
    {
      CANNY = 0,
      SCHARR = 1,
      CURVATURE = 2
    };
    EdgeDetector();
    inline void setCannyLowThreshold(int canny_low_threshold)
    {
      canny_low_threshold_ = canny_low_threshold;
    };
    inline void setCannyHighThreshold(int canny_high_threshold)
    {
      canny_high_threshold_ = canny_high_threshold;
    };
    inline void setCannyKernelSize(int canny_kernel_size)
    {
      if (canny_kernel_size < 3)
      {
        canny_kernel_size = 3;
      }
      else if (canny_kernel_size % 2 != 1)
      {
        canny_kernel_size++;
      }
      canny_kernel_size_ = canny_kernel_size;
    };

    inline void setScharrScale(int scharr_scale)
    {
      scharr_scale_ = scharr_scale;
    };
    inline void setScharrDelta(int scharr_delta)
    {
      scharr_delta_ = scharr_delta;
    };
    inline void setScharrDdepth(int scharr_ddepth)
    {
      scharr_ddepth_ = scharr_ddepth;
    };
    
    inline void setShowEdgesImage(bool show_edges_image)
    {
      show_edges_image_ = show_edges_image;
    };
    inline void setShowInflationImage(bool show_inflation_image)
    {
      show_inflation_image_ = show_inflation_image;
    };
    
    cv::Mat detect(const cv::Mat &src, int method);
    cv::Mat cannyEdges(const cv::Mat &src);
    cv::Mat scharrDerivatives(const cv::Mat &src);
    void inflateEdges(boost::shared_ptr<cv::Mat> edges, int inflation_size);
   private:
    int canny_low_threshold_;
    int canny_high_threshold_;
    int canny_kernel_size_;

    int scharr_scale_;
    int scharr_delta_;
    int scharr_ddepth_;
    
    bool show_edges_image_;
    bool show_inflation_image_;
  };
}  // namespace pandora_slam
