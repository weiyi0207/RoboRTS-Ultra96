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
#include <vector>
#include <cstddef>
#include <numeric>
#include <numeric>
#include <functional>
#include <algorithm>
#include <cmath>
namespace xilinx {
namespace benchmark {
class StatSamples {
 public:
  explicit StatSamples(size_t capacity);
  StatSamples(StatSamples &&other) = default;
  StatSamples(const StatSamples &) = delete;
  StatSamples &operator=(const StatSamples &other) = delete;
  virtual ~StatSamples();

 public:
  void reserve(size_t capacity) { store_.reserve(capacity); }
  void addSample(int value) { store_.push_back(value); }

 public:
  double getMean();
  double getStdVar(const double mean);
  void merge(StatSamples &statSamples);

 private:
  std::vector<int> store_;
};
inline StatSamples::StatSamples(size_t capacity) {
    store_.reserve(capacity);
}

inline StatSamples::~StatSamples() {}

inline double StatSamples::getMean() {
    double sum = std::accumulate(store_.begin(), store_.end(), 0.0);
    return sum / store_.size();
}

inline double StatSamples::getStdVar(const double mean){
    double accum = 0.0;
    std::for_each(store_.begin(), store_.end(),
                  [&](const int d) {
                      accum += (d - mean) * (d - mean);
                  });
    return std::sqrt(accum/store_.size());
}


inline void StatSamples::merge(StatSamples &statSamples) {
  store_.insert(store_.end(), statSamples.store_.begin(),
                statSamples.store_.end());
}
}
}
