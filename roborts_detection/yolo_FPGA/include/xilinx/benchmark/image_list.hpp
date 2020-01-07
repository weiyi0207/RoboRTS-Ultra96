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
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace xilinx {
namespace benchmark {
class ImageList {
 public:
  explicit ImageList(const std::string& filename, bool lazy_load_image);
  ImageList(const ImageList&) = delete;
  ImageList& operator=(const ImageList& other) = delete;
  virtual ~ImageList();

 public:
  const cv::Mat operator[](long i) const;

  std::string to_string();
  void resize_images(int w, int h);

  bool empty() const { return list_.empty(); }
  size_t size() const { return list_.size(); }
  std::string getName(size_t i) const;

 public:
  struct Image {
    std::string name;
    cv::Mat mat;
  };

 private:
  std::vector<Image> list_;
  bool lazy_load_image_;
};

static std::vector<ImageList::Image> get_list(const std::string& filename,
                                              bool lazy_load_image) {
  std::ifstream fs(filename.c_str());
  std::string line;
  auto ret = std::vector<ImageList::Image>{};
  while (getline(fs, line)) {
    // LOG(INFO) << "line = [" << line << "]";
    if (!lazy_load_image) {
      auto image = cv::imread(line);
      if (image.empty()) {
        LOG(WARNING) << "cannot read image: " << line;
      } else {
        ret.emplace_back(ImageList::Image{line, image});
      }
    }else {
      ret.emplace_back(ImageList::Image{line, cv::Mat{}});
    }
  }
  fs.close();
  return ret;
}

inline ImageList::ImageList(const std::string& filename, bool lazy_load_image)
    : list_{get_list(filename, lazy_load_image)},
      lazy_load_image_{lazy_load_image} {}

inline ImageList::~ImageList() {}

inline std::string ImageList::to_string() {
  std::ostringstream str;
  int c = 0;
  for (const auto& image : list_) {
    if (c++ != 0) {
      str << ",";
    }
    str << image.name << image.mat.size();
  }
  // str << "\n";
  return str.str();
}

inline const cv::Mat ImageList::operator[](long i) const {
  const auto& name = list_[i % list_.size()].name;
  const auto& mat = list_[i % list_.size()].mat;
  if (!lazy_load_image_) {
    return mat;
  }
  return cv::imread(name);
}

inline std::string ImageList::getName(size_t i) const {
  const auto& name = list_[i % list_.size()].name;
  return name;
}

inline void ImageList::resize_images(int w, int h) {
  CHECK(!lazy_load_image_) << "only for eager load mode";
  for (auto& image : list_) {
    auto old = cv::Mat(std::move(image.mat));
    cv::resize(old, image.mat, cv::Size{w, h});
  }
}

}
}
