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

  * Filename: openpose.hpp
  *
  * Description:
  * This network is used to detecting poses from a input image.
  *
  * Please refer to document "Xilinx_AI_SDK_User_Guide.pdf" for more details.
  */
#pragma once
#include <opencv2/core.hpp>
#include <memory>

namespace xilinx {
namespace openpose {

/**
 *@struct OpenPoseResult
 *@brief Struct of the result returned by the OpenPoseResult network.
 */
struct OpenPoseResult {
  /// width of input image.
  int width;
  /// height of input image.
  int height;
  /**
   *@struct Line
   *@brief Struct of the line.
   */
  struct Line {
    ///  point A.
    cv::Point2f point_a;
    /// point B.
    cv::Point2f point_b;
  };
  /// a vector of Line.
  std::vector<Line> poses;
};

/**
 * @brief Base class for detecting pose from a input image (cv::Mat).
 *
 * Input a image (cv::Mat).
 *
 * Output is OpenPoseResult.
 *
 * sample code:
 * @code

  auto image = cv::imread(argv[1]);
  if (image.empty()) {
    std::cerr << "cannot load " << argv[1] << std::endl;
    abort();
  }
  auto det = xilinx::openpose::OpenPose::create();
  int width = det->getInputWidth();
  int height = det->getInputHeight();
  cv::Mat res_img;
  cv::resize(image, res_img, cv::Size(width, height));
  auto results = det->run(res_img);
  for(auto &r : results.poses){
    cv::Point2f a = r.point_a;
    cv::Point2f b = r.point_b;
    a.x = a.x * image.cols;
    a.y = a.y * image.rows;
    b.x = b.x * image.cols;
    b.y = b.y * image.rows;
    cv::circle(image, a, 5, cv::Scalar(0, 255, 0), -1);
    cv::circle(image, b, 5, cv::Scalar(0, 255, 0), -1);
    cv::line(image, a, b, cv::Scalar(255, 0, 0), 3, 4);
  }
  cv::imwrite("sample_openpose_result.jpg", image);
  @endcode
 *
 *
 * Display of the openpose model results:
 * @image latex images/sample_openpose_result.jpg "openpose image" width=300px
 */
class OpenPose {
public:
   /**
     * @brief Factory function to get a instance of derived classes of class
     * OpenPose.
     *
     * @param need_mean_scale_process Normalize with mean/scale or not, default value is true.
     * @return An instance of OpenPose class.
     *
     */
  static std::unique_ptr<OpenPose> create(bool need_preprocess = true) {
    return create_ex("openpose_368x368", need_preprocess);
  }

  /**
  * @brief Factory function to get a instance of derived classes of class
  * OpenPose.
  *
  * @param model_name openpose_368x368
  * @param need_mean_scale_process Normalize with mean/scale or not, default
  *value is true.
  * @return An instance of OpenPose class.
  *
  */
  static std::unique_ptr<OpenPose> create_ex(const std::string &model_name,
                                             bool need_preprocess = true);

 public:
  explicit OpenPose();
  OpenPose(const OpenPose&) = delete;
  virtual ~OpenPose();

 public:
 /**
   * @brief Function of get running result of the openpose neuron network.
   *
   * @param img Input data of input image (cv::Mat).
   *
   * @return OpenPoseResult.
   *
   */
  virtual OpenPoseResult run(const cv::Mat& image) = 0;
  /**
   * @brief Function to get InputWidth of the openpose network (input image
   *cols).
   *
   * @return InputWidth of the openpose network
   */
  virtual int getInputWidth() const = 0;
  /**
   *@brief Function to get InputHeigth of the openpose network (input image
   *rows).
   *
   *@return InputHeight of the openpose network.
   */
  virtual int getInputHeight() const = 0;
};
}
}
