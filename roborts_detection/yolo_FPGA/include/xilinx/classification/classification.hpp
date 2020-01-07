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
  * Filename: classification.hpp
  *
  * Description:
  * This network is used to getting classification in the input image
  * Please refer to document "Xilinx_AI_SDK_User_guide.pdf" for more details of these APIs.
  */
#pragma once
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <memory>

namespace xilinx {
namespace classification {
/**
 *@struct ClassificationResult
 *@brief Struct of the result with the classification network.
 *
 */
struct ClassificationResult {
  /**
   *@struct Score
   *@breif Struct of a classification
   */
  struct Score {
    ///  Result's index in ImageNet
    int index;
    ///  Confidence of this category.
    float score;
  };
  /// width of a input image
  int width;
  /// height of a input image
  int height;
  /// all objects, a vector of Score.
  std::vector<Score> scores;
};

/**
 * @brief Classification Network Type
 */
enum Type {
  /// Resnet50 neural network, input size is 224x224
  RESNET_50,
  /// Inception_v1 neural network, input size is 224x224
  INCEPTION_V1,
  /// Inception_v2 neural network, input size is 224x224
  INCEPTION_V2,
  /// Inception_v3 neural network, input size is 299x299
  INCEPTION_V3,
  /// Mobilenet_v2 neural network, input size is 224x224
  MOBILENET_V2,
  /// Resnet50 Tensorflow model, input size is 224x224
  RESNET_50_TF,
  /// Inception_v1 Tensorflow model, input size is 224x224
  INCEPTION_V1_TF,
  /// Mobilenet_v2 Tensorflow model, input size is 224x224
  MOBILENET_V2_TF,

  NUM_OF_TYPE
};

/**
 * @brief Base class for detecting objects in the input image (cv::Mat).
 *
 * Input is a image (cv::Mat).
 *
 * Output is index and score of objects in the input image.
 * *
 * sample code:
 * @code
   auto image = cv::imread("test.jpg");
   auto network = xilinx::classification::Classification::create(
                  xilinx::classification::RESNET_50,
                  true);
   auto result = network->run(image);
   for (const auto &r : result.scores) {
      auto score = r.score;
      auto index = network->lookup(r.index);
   }
   @endcode
 *
 */

class Classification {
 public:
  /**
   * @brief Factory function to get a instance of derived classes of class
   * Classification.
   *
   * @param type Enum Type
   * @param @param need_mean_scale_process Normalize with mean/scale or not,
   *default value is true.
   * @return An instance of Classification class.
   *
   */
  static std::unique_ptr<Classification> create(
      Type type, bool need_preprocess = true);
  /**
   * @brief Factory function to get a instance of derived classes of class
   * Classification.
   *
   * @param type string
   * @param @param need_mean_scale_process Normalize with mean/scale or not,
   *default value is true.
   * @return An instance of Classification class.
   *
   */
  static std::unique_ptr<Classification> create_ex(
      const std::string& model_name, bool need_preprocess = true);

  /**
   * @brief Get the classification corresponding by index
   * @param index , the network result
   * @return classification
   */
  static const char* lookup(int index);

 public:
  explicit Classification();
  Classification(const Classification&) = delete;
  virtual ~Classification();

 public:
 /**
  * @brief Function of get running result of the Classification neuron network.
  *
  * @param img Input data of input image (cv::Mat).
  *
  * @return ClassificationResult.
  *
  */
  virtual ClassificationResult run(const cv::Mat& image) = 0;

  /**
   * @brief Function to get InputWidth of the classification network (input image
   *cols).
   *
   * @return InputWidth of the classification network
   */
  virtual int getInputWidth() const = 0;
  /**
   *@brief Function to get InputHeigth of the classification network (input image
   *rows).
   *
   *@return InputHeight of the classification network.
   */
  virtual int getInputHeight() const = 0;
};
}
}
