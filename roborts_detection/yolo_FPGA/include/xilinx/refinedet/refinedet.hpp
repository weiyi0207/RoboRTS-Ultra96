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
  * Filename: refinedet.hpp
  *
  * Description:
  * This network is used to getting position and score of objects in the input image
  * Please refer to document "Xilinx_AI_SDK_User_Guide.pdf" for more details of these APIs.
  */
#pragma once
#include <opencv2/core.hpp>
#include <memory>

namespace xilinx {
namespace refinedet {
/**
 *@struct RefineDetResult
 *@brief Struct of the result with the refinedet network.
 *
 */
struct RefineDetResult {
    /// width of the input image.
    int width;
    /// height of the input image.
    int height;
    /**
     *@struct BoundingBox
     *@brief Struct of a object coordinate and confidence.
     */
    struct BoundingBox{
      /// x-coordinate , x is normalized relative to the input image cols ,the
      /// value range from 0 to 1.
      float x;
      /// y-coordinate , y is normalized relative to the input image rows ,the
      /// value range from 0 to 1.
      float y;
      /// body width , width is normalized relative to the input image cols ,
      /// the value range from 0 to 1.
      float width;
      /// body height , heigth is normalized relative to the input image rows ,
      /// the value range from 0 to 1.
      float height;
      /// body detection confidence, the value range from 0 to 1.
      float score;
    };
    /// the vector of BoundingBox.
    std::vector<BoundingBox> bboxes;
};

/**
  * @brief RefineDet Network Type
  */
enum Type {
  /// Input size is 480x360
  REFINEDET_480x360,
  /// Input size is 480x360, OPS is 10g
  REFINEDET_480x360_10G,
  /// Input size is 480x360, OPS is 5g
  REFINEDET_480x360_5G,
  /// Input size is 640x480
  REFINEDET_640x480
};

/**
 * @brief Base class for detecting pedestrian in the input image (cv::Mat).
 *
 * Input is a image (cv::Mat).
 *
 * Output is position and score of pedestrian in the input image.
 *
 * sample code:
 * @code
   auto image = cv::imread("sample_refinedet.jpg");
   auto network = deephi::refinedet::RefineDet::create(
                  deephi::refinedet::RefineDet::REFINEDET_640x480,
                  true);
   auto results = det->run(image);
   for (const auto &r : results.bboxes) {
      auto score = r.score;
      auto x = r.x * image.cols;
      auto y = r.y * image.rows;
      auto width = r.width * image.cols;
      auto height = r.height * image.rows;
   }
   @endcode
 *
 * Display of the refinedet_REFINEDET_640x480 model results:
 * @image latex images/sample_refinedet_REFINEDET_640x480_result.jpg "REFINEDET_640x360 detect result" width=\textwidth
 *
 */
class RefineDet {
 public:
   /**
   * @brief Factory function to get a instance of derived classes of class
   * RefineDet.
   *
   * @param type Enum Type
   * @param @param need_mean_scale_process Normalize with mean/scale or not,
   *default value is true.
   * @return An instance of RefineDet class.
   *
   */
  static std::unique_ptr<RefineDet> create(Type type = REFINEDET_480x360,
                                           bool need_preprocess = true);

  /**
  * @brief Factory function to get a instance of derived classes of class
  * RefineDet.
  * @note for internal use
  * @param type string
  * @param @param need_mean_scale_process Normalize with mean/scale or not,
  *default value is true.
  * @return An instance of RefineDet class.
  *
  */
  static std::unique_ptr<RefineDet> create_ex(const std::string& model_name,
                                              bool need_preprocess = true);

 public:
  explicit RefineDet();
  RefineDet(const RefineDet&) = delete;
  virtual ~RefineDet();

 public:
 /**
  * @brief Function of get running result of the RefineDet neuron network.
  *
  * @param img Input data of input image (cv::Mat).
  *
  * @return A vector of RefineDetResult.
  *
  */

  virtual RefineDetResult run(const cv::Mat& image) = 0;
  /**
   * @brief Function to get InputWidth of the refinedet network (input image cols).
   *
   * @return InputWidth of the refinedet network
   */
  virtual int getInputWidth() const = 0;
  /**
   *@brief Function to get InputHeigth of the refinedet network (input image rows).
   *
   *@return InputHeight of the refinedet network.
   */
  virtual int getInputHeight() const = 0;
};
}
}
