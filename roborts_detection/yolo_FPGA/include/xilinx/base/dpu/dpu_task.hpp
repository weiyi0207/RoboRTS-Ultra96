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
#include <memory>
#include <vector>
#include <opencv2/core.hpp>
namespace xilinx {
namespace base {
struct DpuLayerData {
  int8_t* addr_;
  size_t size_;
  float scale_;
  size_t width_;
  size_t height_;
  size_t channel_;
};
using DpuLayerInput = DpuLayerData;
using DpuLayerOutput = DpuLayerData;
class DpuTask {
 public:
  DpuTask();
  virtual ~DpuTask();
  DpuTask(const DpuTask& other) = delete;
  DpuTask& operator=(const DpuTask& rhs) = delete;

 public:
  static std::unique_ptr<DpuTask> create(
      const std::string& kernal_name,         //
      std::vector<std::string> input_layers,  //
      std::vector<std::string> output_layers  //
      );

 public:
  virtual void run() = 0;
  // by default, no mean-scale processing, after invoking this
  // function, mean-scale processing is enabled. You cannot turn it
  // off after enabling.
  virtual void setMeanScaleBGR(const std::vector<float> &mean,
                               const std::vector<float> &scale) = 0;
  // TODO: too many interface methods
  virtual void setImageBGR(const cv::Mat &img) = 0;
  virtual void setImageRGB(const cv::Mat &img) = 0;

  virtual std::vector<float> getMean() = 0;
  virtual std::vector<float> getScale() = 0;
  // FROM DpuObject, TODO: consuming name with getInputLayerData()
  virtual std::vector<DpuLayerInput> getLayerInputData() = 0;
  virtual std::vector<DpuLayerOutput> getLayerOutputData() = 0;
protected:
  // virtual DpuLayerInput getInputLayerData(const std::string &layer) = 0;
  // virtual DpuLayerOutput getOutputLayerData(const std::string &layer) = 0;

};
}
}

// Local Variables:
// mode:c++
// c-basic-offset: 2
// coding: undecided-unix
// End:
