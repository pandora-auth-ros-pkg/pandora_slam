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
    low_threshold_ = 20;
    ratio_ = 3;
    kernel_size_ = 3;
    window_name_ = "Edge Map";
  }

  cv::Mat EdgeDetector::detect(const cv::Mat &src)
  {
    cv::Mat dst, detected_edges;

    /// Reduce noise with a kernel 3x3
    cv::blur(src, detected_edges, cv::Size(3,3));
    /// Canny detector
    cv::Canny(detected_edges, detected_edges, low_threshold_,
      low_threshold_ * ratio_, kernel_size_ );
    /// Create a matrix of the same type and size as src (for dst)
    dst.create(src.size(), src.type());
    /// Using Canny's output as a mask, we display our result
    dst = cv::Scalar::all(0);

    src.copyTo(dst, detected_edges);
    //~ cv::imshow(window_name_, dst);
    //~ cv::imshow("Source image", src);
    //~ cv::waitKey(1);
    
    return detected_edges;
   }
}  // namespace pandora_slam
