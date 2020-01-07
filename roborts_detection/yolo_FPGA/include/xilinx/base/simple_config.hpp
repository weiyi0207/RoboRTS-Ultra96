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
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
namespace xilinx{
namespace base{

class SimpleConfig {
public:

  static std::shared_ptr<SimpleConfig> getOrCreateSimpleConfig(const std::string& filename);

  SimpleConfig(const std::string &filename);

  template <typename T> T as(const std::string &name) const;

  struct SimpleConfigViewer {
    SimpleConfigViewer(const SimpleConfig &cfg, const std::string &name);

    template <class T> SimpleConfigViewer operator[](const T &name) const;

    SimpleConfigViewer operator()(const std::string &name) const;

    SimpleConfigViewer operator()(int index) const;
    std::vector<SimpleConfigViewer> fields() const;

    template <typename T> T as() const;

    const SimpleConfig &cfg_;
    std::string name_;
  };

  struct SimpleConfigViewer operator()(const std::string &name =
                                       std::string()) const;

private:
  std::map<std::string, std::string> values_;
  std::vector<std::string> fields_;
  friend struct SimpleConfigViewer;
  /* following are private help functions*/
private:

  void Initialize(const std::string &filename);


  template <typename T>
  static void ParseValue(const std::string &text, T &value);

};
}
}
