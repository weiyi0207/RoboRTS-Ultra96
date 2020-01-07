/*
-- (c) Copyright 2018 Xilinx, Inc. All rights reserved.
--
-- This file contains confidential and proprietary information
-- of Xilinx, Inc. and is protected under U.S. and
-- international copyright and other intellectual property
-- laws.
--
-- DISCLAIMER
-- This disclaimer is not a license and does not grant any
-- rights to the materials distributed herewith. Except as
-- otherwise provided in a valid license issued to you by
-- Xilinx, and to the maximum extent permitted by applicable
-- law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
-- WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
-- AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
-- BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
-- INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
-- (2) Xilinx shall not be liable (whether in contract or tort,
-- including negligence, or under any other theory of
-- liability) for any loss or damage of any kind or nature
-- related to, arising under or in connection with these
-- materials, including for any direct, or any indirect,
-- special, incidental, or consequential loss or damage
-- (including loss of data, profits, goodwill, or any type of
-- loss or damage suffered as a result of any action brought
-- by a third party) even if such damage or loss was
-- reasonably foreseeable or Xilinx had been advised of the
-- possibility of the same.
--
-- CRITICAL APPLICATIONS
-- Xilinx products are not designed or intended to be fail-
-- safe, or for use in any application requiring fail-safe
-- performance, such as life-support or safety devices or
-- systems, Class III medical devices, nuclear facilities,
-- applications related to the deployment of airbags, or any
-- other applications that could lead to death, personal
-- injury, or severe property or environmental damage
-- (individually and collectively, "Critical
-- Applications"). Customer assumes the sole risk and
-- liability of any use of Xilinx products in Critical
-- Applications, subject only to applicable laws and
-- regulations governing limitations on product liability.
--
-- THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
-- PART OF THIS FILE AT ALL TIMES.
*/

/*
 * Filename: roadline.hpp
 *
 * Description:
 * This network is used to detecting road line
 *
 * Please refer to document "Xilnx_AI_SDK_User_Guide.pdf" for more details of
 *these APIs.
 */

#pragma once
#include <opencv2/core.hpp>
#include <memory>

namespace xilinx {
namespace roadline {
/**
 *@struct RoadLineResult
 *@brief Struct of the result returned by the roadline network.
 */
struct RoadLineResult{
  /// width of input image.
  int width;
  /// height of input image.
  int height;
  /**
   *@struct Line
   *@brief Struct of the result returned by the roadline network.
   */
  struct Line {
    /// road line type, the value range from 0 to 3.
    /// \li \c 0 : background
    /// \li \c 1 : white dotted line
    /// \li \c 2 : white solid line
    /// \li \c 3 : yollow line
    int type;
    /// point clusters, make line from these.
    std::vector<cv::Point> points_cluster;
  };
  /// the vector of line
  std::vector<Line> lines;
};


/**
 * @brief Base class for detecting roadline from a image (cv::Mat).
 *
 * Input is a image (cv::Mat).
 *
 * Output road line type and points maked road line.
 *
 * @note The input image size is 640x480
 *
 * sample code:
 * @code
    auto det = xilinx::roadline::RoadLine::create();
    auto image = cv::imread(argv[1]);
    if(image.empty()) {
        cerr << "cannot load " << argv[1] << endl;
        abort();
    }

    std::vector<int> color1 = {0, 255, 0, 0, 100, 255};
    std::vector<int> color2 = {0, 0, 255, 0, 100, 255};
    std::vector<int> color3 = {0, 0, 0, 255, 100, 255};

    RoadLineResult results = det->run(image);
    for(auto line : result.lines){
      std::vector<cv::Point> points_poly = line.points_cluster;
      int type == line.type;
      if(type == 2 && points_poly[0].x < image.rows * 0.5)
          continue;
       cv::polylines(image, points_poly, false, Scalar(color1[type], color2[type], color3[type]), \
	3, CV_AA, 0);
    }
    cv::imwrite("results.jpg",image);
    @endcode
 *
 * Display of the roadline model results:
 * @image latex images/sample_roadline_result.jpg "roadline result image" width=300px
 *
 */
class RoadLine {
public:
  /**
   * @brief Factory function to get a instance of derived classes of class
   *RoadLine.
   *
   * @param need_process  normalize with mean/scale or not, default value is
   *true.
   *
   * @return An instance of RoadLine class.
   */

  static std::unique_ptr<RoadLine> create(bool need_preprocess = true);

protected:
   explicit RoadLine();
   RoadLine(const RoadLine &) = delete;

 public:
    virtual ~RoadLine();

 public:
  /**
   * @brief Function to get InputWidth of the roadline network (input image cols).
   *
   * @return InputWidth of the roadline network.
   */
  virtual int getInputWidth() const = 0;
 /**
   * @brief Function to get InputHight of the roadline network (input image rows).
   *
   * @return InputHeight of the roadline network.
   */
  virtual int getInputHeight() const = 0;

   /**
   * @brief Function of get running result of the RoadLine network.
   *
   * @param img Input data , input image (cv::Mat) need to resized as 640x480.
   *
   * @return The struct of RoadLineResult
   */
   virtual RoadLineResult run(const cv::Mat &image) = 0;
};
}
}
