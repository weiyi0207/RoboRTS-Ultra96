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
  * Filename: posedetect.hpp
  *
  * Description:
  * This network is used to detecting a pose from a input image.
  *
  * Please refer to document "Xilinx_AI_User_Guide.pdf" for more details of these APIs.
  */
#pragma once
#include <opencv2/core.hpp>
#include <memory>
#include <vector>

namespace xilinx {
namespace posedetect {

/**
 *@struct PoseDetectResult
 *@brief Struct of the result returned by the posedetect network.
 */
struct PoseDetectResult {
  /// width of input image.
  int width;
  /// height of input image.
  int height;
  /**
  *@struct Point
  *@brief Struct of a coordinate point
  */
  struct Point {
    /// x-coordinate, x is normalized relative to the input image cols ,the
    /// value range from 0 to 1.
    float x;
    /// y-coordinate, y is normalized relative to the input image rows ,the
    /// value range from 0 to 1.
    float y;
  };
  /// a pose , represented by 14 coordinate points.
  /// 1: R_shoulder, 2: R_elbow, 3: R_wrist, 4: L_shoulder, 5: L_elbow, 6: L_wrist, 7: R_hip,
  /// 8: R_knee, 9: R_ankle, 10: L_hip, 11: L_knee, 12: L_ankle, 13: head, 14: neck
  using Pose14Pt = std::array<Point, 14>;
  /// the pose of input image.
  Pose14Pt pose14pt;
};

/**
 * @brief Base class for detecting a pose from a input image (cv::Mat).
 * @note support detect a signle pose.
 *
 * Input a image (cv::Mat).
 *
 * Output is PoseDetectResult.
 *
 * sample code:
 * @code
   auto det = xilinx::posedetect::PoseDetect::create();
   auto image = cv::imread("sample.jpg");
   auto results = det->run(image);
   for(auto result: results.pose14pt) {
       std::cout << result << std::endl;
   }

   @endcode

 * Display of the posedetect model results:
 * @image latex images/sample_posedetect_result.jpg "pose detect image" width=\textwidth
 *
 */

class PoseDetect {
public:
    /**
     * @brief Factory function to get a instance of derived classes of class
     * PoseDetect.
     *
     * @param need_mean_scale_process Normalize with mean/scale or not, default value is true.
     * @return An instance of PoseDetect class.
     *
     */
   static std::unique_ptr<PoseDetect> create(bool need_preprocess = true);

 public:
   explicit PoseDetect();
   PoseDetect(const PoseDetect &) = delete;
   virtual ~PoseDetect();

 public:
   /**
  * @brief Function to get InputWidth of the PoseDetect network (input image cols).
  *
  * @return InputWidth of the PoseDetect network.
  */
   virtual int getInputWidth() const = 0;
   /**
   *@brief Function to get InputHeigth of the PoseDetect network (input image rows).
   *
   *@return InputHeight of the PoseDetect network.
   */

   virtual int getInputHeight() const = 0;
   /**
   * @brief Function of get running result of the posedetect neuron network.
   *
   * @param img Input data of input image (cv::Mat).
   *
   * @return PoseDetectResult.
   *
   */
   virtual PoseDetectResult run(const cv::Mat &image) = 0;

};
}
}
