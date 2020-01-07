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
  * Copyright (c) 2016-2018 Xilinx Tech, Inc.
  *
  * All Rights Reserved. No part of this source code may be reproduced
  * or transmitted in any form or by any means without the prior written
  * permission of Xilinx Tech, Inc.
  *
  * Filename: facedetect.hpp
  *
  * Description:
  * This network is used to getting position and score of faces in the input image
  * Please refer to document "XILINX_AI_SDK_Programming_Guide.pdf" for more details of these APIs.
  */
#pragma once
#include <opencv2/core.hpp>
#include <memory>
namespace xilinx {
namespace facedetect {

/**
 *@struct FaceDetectResult
 *@brief Struct of the result with the facedetect network.
 *
 */
struct FaceDetectResult {
  /**
   *@struct BoundingBox
   *@brief Struct of a face coordinate and confidence.
   */
  struct BoundingBox {
    /// x-coordinate , x is normalized relative to the input image cols ,the
    /// value range from 0 to 1.
    float x;
    /// y-coordinate , y is normalized relative to the input image rows ,the
    /// value range from 0 to 1.
    float y;
    /// face width , width is normalized relative to the input image cols , the
    /// value range from 0 to 1.
    float width;
    /// face height , heigth is normalized relative to the input image rows ,
    /// the value range from 0 to 1.
    float height;
    /// face confidence, the value range from 0 to 1.
    float score;
  };
  /// width of a input image
  int width;
  /// height of a input image
  int height;
  /// all faces, a vector of BoundingBox
  std::vector<BoundingBox> rects;
};

/**
 * @brief Facedetect Network Type
 */
enum Type {
  /// Input size is 320x320.
  DENSE_BOX_320x320,
  /// Input size is 640x320.
  DENSE_BOX_640x360
};

/**
 * @brief Base class for detecting the position of faces in the input image (cv::Mat).
 *
 * Input is a image (cv::Mat).
 *
 * Output is a vector of position and score infomation for faces in the input image.
 *
 * sample code:
 * @code
   auto image = cv::imread("test_faces.jpg");
   auto network = xilinx::facedetect::FaceDetect::create(
                  xilinx::facedetect::DENSE_BOX_640x360,
                  true);
   auto result = network->run(image);
   for (const auto &r : result) {
      auto score = r.score;
      auto x = r.x * image.cols;
      auto y = r.y * image.rows;
      auto width = r.width * image.cols;
      auto height = rz.height * image.rows;
   }
   @endcode
 *
 * Display of the facedetect model results:
 * @image latex images/sample_facedetect_result.jpg "facedetect result image" width=\textwidth
 *
 */
class FaceDetect {
public:
  /**
   * @brief Factory function to get a instance of derived classes of class
   * FaceDetect.
   *
   * Support 2 types of input size:
   * 1. width = 640 and height = 360
   * 2. width = 320 and height = 320
   *
   * @param type DENSE_BOX_320 or DENSE_BOX_640
   * @param need_mean_scale_process Normalize with mean/scale or not, default value is true.
   * @return An instance of FaceDetect class.
   *
   */
  static std::unique_ptr<FaceDetect> create(Type type,
                                        bool need_preprocess = true) {
    std::string model_name = "dense_box_640x360";
    switch(type) {
      case DENSE_BOX_320x320:
        model_name = "dense_box_320x320";
        break;
      case DENSE_BOX_640x360:
        model_name = "dense_box_640x360";
        break;
      }
    return create_ex(model_name, need_preprocess);
  }

  /**
   * @brief Factory function to get instance of derived classes of class
   FaceDetect
   * @note for internal use
   *
   * @param modle_name Model name
   * @param need_mean_scale_process Normalize with mean/scale or not, default
   value is true.
   * @return An instance of FaceDetect class.
   */
  static std::unique_ptr<FaceDetect> create_ex(
      const std::string &model_name, bool need_preprocess = true);

 protected:
  explicit FaceDetect();
  FaceDetect(const FaceDetect &) = delete;
  FaceDetect &operator=(const FaceDetect &) = delete;

public:
  virtual ~FaceDetect();

  /**
   * @brief Function to get InputWidth of the facedetect network (input image cols).
   *
   * @return InputWidth of the facedetect network
   */
  virtual int getInputWidth() const = 0;

  /**
   *@brief Function to get InputHeigth of the facedetect network (input image rows).
   *
   *@return InputHeight of the facedetect network.
   */
  virtual int getInputHeight() const = 0;

  /**
   * @brief Function to get detect threshold.
   * @return detect threshold , the value range from 0 to 1.
   */
  virtual float getThreshold() const = 0;

  /**
   * @brief Function of set detect threshold.
   * @note The results will filter by detect threshold (score > threshold).
   * @param threshold , the value range from 0 to 1.
   */
  virtual void setThreshold(float threshold) = 0;

  /**
   * @brief Function of get running result of the facedetect network.
   *
   * @param img Input Data , input image (cv::Mat) need to be resized to
   *InputWidth and InputHeight required by the network.
   *
   * @return the results of the face detect network , a collection of
   *FaceDetectResult filter by score >= det_threshold
   *
   */
  virtual FaceDetectResult run(const cv::Mat &img) = 0;
};

/*!@} */
}
}


// Local Variables:
// mode:c++
// c-basic-offset: 2
// coding: utf-8-unix
// End:
