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
#pragma once
#include <opencv2/core.hpp>
#include <memory>
#include "./xilinx/config/XILINX_AI_SDK.pb.h"
#include "./dpu/dpu_task.hpp"
namespace xilinx {
namespace base {
class Base {
 public:
  static std::unique_ptr<Base> create(const std::string& model_name,
                                      bool need_preprocess = true);

 public:
  explicit Base();
  Base(const Base&) = delete;
  virtual ~Base();

 public:
  virtual void setInputImageBGR(const cv::Mat & image) = 0;
  virtual void setInputImageRGB(const cv::Mat & image) = 0;
  virtual void execute(int task_index) = 0;
  virtual const xilinx::config::DpuModel& getConfig() const = 0;
  virtual const std::vector<DpuLayerInput>& getDpuLayerInput(int task_index)
      const = 0;
  virtual const std::vector<DpuLayerOutput>& getDpuLayerOutput(int task_index)
      const = 0;
  /**
   * @brief Function to get InputWidth of the facedetect network (input image
   *cols).
   *
   * @return InputWidth of the facedetect network
   */
  virtual int getInputWidth() const = 0;
  /**
   *@brief Function to get InputHeigth of the facedetect network (input image
   *rows).
   *
   *@return InputHeight of the facedetect network.
   */
  virtual int getInputHeight() const = 0;
};

template<typename Interface>
class WithBase : public Interface {
 public:
  explicit WithBase(const std::string& model_name, bool need_preprocess = true)
      : base_{Base::create(model_name, need_preprocess)} {};
  WithBase(const WithBase&) = delete;
  virtual ~WithBase() {};
  // assuming implement Interface::getInputHeight()
  virtual int getInputWidth() const override {
    return base_->getInputWidth();
  }
  virtual int getInputHeight() const override {
    return base_->getInputHeight();
  }
 protected:
  std::unique_ptr<Base> base_;
};
}
}
