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
#ifndef DPMATH_SSD_NORMALIZER_HPP_
#define DPMATH_SSD_NORMALIZER_HPP_

#include <cstdint>

namespace xilinx {
namespace dpmath {

class SSDNormalizer {
 public:
  SSDNormalizer(bool across_spatial, bool channel_shared,
      int height, int width, int channel, int output_fix_pos, float eps);
  SSDNormalizer(bool across_spatial, bool channel_shared,
      int height, int width, int channel, int output_fix_pos);

  virtual ~SSDNormalizer();

  void loadScaleParam(const int8_t* scale, int scale_fix_pos);
  void loadScaleParam(const float* scale);

  template<typename T>
  void normalize(const int8_t* input, T* output);

  int normalize_neon(const int8_t* input, int8_t* output);

 protected:
  static constexpr float EPS = 1e-8;

  bool across_spatial_;
  bool channel_shared_;

  int height_;
  int width_;
  int channel_;

  int output_fix_pos_;

  float eps_;

  int spatial_dim_;
  int num_;

  float* scale_; // initialization only once
  float* norm_buf_;
  float* scale_buf_;

};

}
}

#endif

