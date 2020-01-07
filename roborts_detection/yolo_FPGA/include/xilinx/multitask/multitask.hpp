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
 * Filename: multitask.hpp
 *
 * Description:
 * This module implement MultiTask Network for ADAS, include detection,
 *segmentation and car towards angle;
 *
 * Please refer to document "xilinx_XILINX_AI_SDK_user_guide.pdf" for more details of
 *these APIs.
 */
#pragma once
#include <opencv2/core.hpp>
#include <memory>

namespace xilinx {
namespace multitask {
/**
 *@struct VehicleResult
 *@brief A struct to define detection result of MultiTask
 */
struct VehicleResult {
  // number of classes
  // label: 0 name: "background"
  // label: 1 name: "person"
  // label: 2 name: "car"
  // label: 3 name: "truck"
  // label: 4 name: "bus"
  // label: 5 name: "bike"
  // label: 6 name: "sign"
  // label: 7 name: "light"
  int label;
  // confidence of this target
  float score;
  // x-coordinate, x is normalized relative to the input image cols ,the value ,
  // the range from 0 to 1.
  float x;
  // y-coordinate ,y is normalized relative to the input image rows ,the value
  // range from 0 to 1.
  float y;
  // width, width is normalized relative to the input image cols ,the value
  // range from 0 to 1.
  float width;
  // height, height is normalized relative to the input image rows ,the value
  // range from 0 to 1.
  float height;
  // the angle between the target vehicle and ourself;
  float angle;
};

/**
 *@struct MultiTaskResult2
 *@brief  Struct of the result returned by the MultiTask network, when you need
 *to visualize.
 */
struct MultiTaskResult {
  // width of input image.
  int width;
  // height of input image.
  int height;
  // Detection result of SSD task
  std::vector<VehicleResult> vehicle;
  // segmentation  result to visualize , cv::Mat type is CV_8UC1 or CV_8UC3.
  cv::Mat segmentation;
};

/**
 * @brief Base class for ADAS MuiltTask from a image (cv::Mat).
 *
 * Input a image (cv::Mat).
 *
 * Output is struct MultiTaskResult include segmentation results, detection
 detection results and vehicle towards;
 *
 *
 *
 * sample code:
 * @code
    auto det = xilinx::multitask::MultiTask::create();
    auto image = cv::imread("sample_multitask.jpg");
    auto result = det->run_8UC3(image);
    cv::imwrite("res.jpg",result.segmentation);
   @endcode
 *
 *
 * Display of the multitask model results:
 * @image latex images/sample_multitask_visualization_result.jpg "multitask visualization result image" width=\textwidth
 *
 */
class MultiTask {
 public:
  /**
  * @brief Factory function to get a instance of derived classes of class
  *MuliTask.
  *
  * @param need_perprocess  normalize with mean/scale or not, default value is
  *true.
  *
  * @return An instance of MultiTask class.
  */
  static std::unique_ptr<MultiTask> create(bool need_preprocess = true) {
    return create_ex("multitask", need_preprocess);
  }
  static std::unique_ptr<MultiTask> create_ex(const std::string& model_name,
                                              bool need_preprocess = true);

 protected:
  explicit MultiTask();
  MultiTask(const MultiTask&) = delete;

 public:
  virtual ~MultiTask();

 public:
  /**
   * @brief Function to get InputWidth of the multitask network (input image
   *cols).
   *
   * @return InputWidth of the multitask network.
   */
  virtual int getInputWidth() const = 0;
  /**
   * @brief Function to get InputHight of the multitask network (input image
   *rows).
   *
   * @return InputHeight of the multitask network.
   */
  virtual int getInputHeight() const = 0;

  /**
  * @brief Function of get running result from the MultiTask network.
  * @note The type is CV_8UC1 of the MultiTaskResult.segmentation.
  * @param image Input image
  * @return The struct of MultiTaskResult
  */
  virtual MultiTaskResult run_8UC1(const cv::Mat& image) = 0;

  /**
  * @brief Function of get running result from the MultiTask network.
  * @note The type is CV_8UC3 of the MultiTaskResult.segmentation.
  *@param image Input image;
  * @return The struct of MultiTaskResult
  */
  virtual MultiTaskResult run_8UC3(const cv::Mat& image) = 0;
};

/**
 * @brief Base class for ADAS MuiltTask8UC1 from a image (cv::Mat).
 *
 * Input a image (cv::Mat).
 * Output is struct MultiTaskResult include segmentation results, detection
 results and vehicle towards; The result cv::Mat type is CV_8UC1
 *
 * sample code:
 * @code
    auto det = xilinx::multitask::MultiTask8UC1::create();
    auto image = cv::imread("sample_multitask.jpg");
    auto result = det->run(image);
    cv::imwrite("res.jpg",result.segmentation);
   @endcode
  */
class MultiTask8UC1
{
public:
  /**
   * @brief Factory function to get a instance of derived classes of class
   *MuliTask8UC1.
   *
   * @param need_perprocess  normalize with mean/scale or not, default value
   *is true.
   *
   * @return An instance of MultiTask8UC1 class.
   */
  static std::unique_ptr<MultiTask8UC1> create(bool need_preprocess = true) {
    return create_ex("multitask", need_preprocess);
  }
  static std::unique_ptr<MultiTask8UC1> create_ex(const std::string &model_name,
                                                  bool need_preprocess = true) {
    return std::unique_ptr<MultiTask8UC1>(new MultiTask8UC1(
        MultiTask::create_ex(model_name, need_preprocess)));
  }

protected:
  explicit MultiTask8UC1(std::unique_ptr<MultiTask> multitask)
      : multitask_{ std::move(multitask) } {}
  MultiTask8UC1(const MultiTask8UC1 &) = delete;

public:
  virtual ~MultiTask8UC1() {}

public:
  /**
    * @brief Function to get InputWidth of the multitask network (input image
    *cols).
    *
    * @return InputWidth of the multitask network.
    */
  virtual int getInputWidth() const { return multitask_->getInputWidth(); }
  /**
   * @brief Function to get InputHight of the multitask network (input image
   *rows).
   *
   * @return InputHeight of the multitask network.
   */
  virtual int getInputHeight() const { return multitask_->getInputHeight(); }

  /**
  * @brief Function of get running result from the MultiTask network.
  * @note The type is CV_8UC1 of the MultiTaskResult.segmentation.
  *
  * @param image Input image
  * @return The struct of MultiTaskResult
  */
  virtual MultiTaskResult run(const cv::Mat &image) {
    return multitask_->run_8UC1(image);
  }

private:
  std::unique_ptr<MultiTask> multitask_;
};


/**
 * @brief Base class for ADAS MuiltTask8UC3 from a image (cv::Mat).
 *
 * Input a image (cv::Mat).
 * Output is struct MultiTaskResult include segmentation results, detection
 results and vehicle towards; The result cv::Mat type is CV_8UC3
 *
 * sample code:
 * @code
    auto det = xilinx::multitask::MultiTask8UC3::create();
    auto image = cv::imread("sample_multitask.jpg");
    auto result = det->run(image);
    cv::imwrite("res.jpg",result.segmentation);
   @endcode
  */
class MultiTask8UC3
{
public:
  /**
   * @brief Factory function to get a instance of derived classes of class
   *MuliTask8UC3.
   *
   * @param need_perprocess  normalize with mean/scale or not, default value
   *is true.
   *
   * @return An instance of MultiTask8UC3 class.
   */
  static std::unique_ptr<MultiTask8UC3> create(bool need_preprocess = true) {
    return create_ex("multitask", need_preprocess);
  }
  static std::unique_ptr<MultiTask8UC3> create_ex(const std::string &model_name,
                                                  bool need_preprocess = true) {
    return std::unique_ptr<MultiTask8UC3>(new MultiTask8UC3(
        MultiTask::create_ex(model_name, need_preprocess)));
  }

protected:
  explicit MultiTask8UC3(std::unique_ptr<MultiTask> multitask)
      : multitask_{ std::move(multitask) } {}
  MultiTask8UC3(const MultiTask8UC3 &) = delete;

public:
  virtual ~MultiTask8UC3() {}

public:
  /**
    * @brief Function to get InputWidth of the multitask network (input image
    *cols).
    *
    * @return InputWidth of the multitask network.
    */
  virtual int getInputWidth() const { return multitask_->getInputWidth(); }
  /**
   * @brief Function to get InputHight of the multitask network (input image
   *rows).
   *
   * @return InputHeight of the multitask network.
   */
  virtual int getInputHeight() const { return multitask_->getInputHeight(); }

  /**
  * @brief Function of get running result from the MultiTask network.
  * @note The type is CV_8UC3 of the MultiTaskResult.segmentation.
  *
  * @param image Input image
  * @return The struct of MultiTaskResult
  */
  virtual MultiTaskResult run(const cv::Mat &image) {
    return multitask_->run_8UC3(image);
  }

private:
  std::unique_ptr<MultiTask> multitask_;
};



}
}
