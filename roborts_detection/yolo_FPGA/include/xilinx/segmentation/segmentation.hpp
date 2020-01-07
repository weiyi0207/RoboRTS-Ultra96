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
  * Filename:segmentation.hpp
  *
  * Description:
  * Segmentation for ADAS
  *
  * Please refer to document "Xilinx_AI_SDK_User_Guide.pdf" for more
  *details of these APIs.
  */
#pragma once
#include <opencv2/core.hpp>
#include <memory>

namespace xilinx {
namespace segmentation {
/**
 *@struct SegmentationResult
 *@brief Struct of the result returned by the segementation network.
 */
struct SegmentationResult {
    //width of input image;
	int width;
    //height of input image;
	int height;
    //segmentation result; The cv::Mat type is CV_8UC1 or CV_8UC3
    cv::Mat segmentation;

};

/**
 *@brief Segmentation Network Type , declaration Segmentation Network
 */
enum Type {
  /// FPN
  /// num of segmentation classes
  /// label 0 name: "unlabeled"
  /// label 1 name: "ego vehicle"
  /// label 2 name: "rectification border"
  /// label 3 name: "out of roi"
  /// label 4 name: "static"
  /// label 5 name: "dynamic"
  /// label 6 name: "ground"
  /// label 7 name: "road"
  /// label 8 name: "sidewalk"
  /// label 9 name: "parking"
  /// label 10 name: "rail track"
  /// label 11 name: "building"
  /// label 12 name: "wall"
  /// label 13 name: "fence"
  /// label 14 name: "guard rail"
  /// label 15 name: "bridge"
  /// label 16 name: "tunnel"
  /// label 17 name: "pole"
  /// label 18 name: "polegroup"
  FPN,
  /// ENET, not support yet.
  ENET,
  /// ESPNET , not support yet.
  ESPNET
};
/**
 * @brief Base class for Segmentation.
 *
 * Input is a image (cv:Mat).
 *
 * Output is struct SegmentationResultShow define before.
 *
 * sample code :
   @code
    auto det = xilinx::segmentation::Segmentation::create(xilinx::segmentation::FPN);
    auto img = cv::imread("sample_segmentation.jpg");

    int width = det->getInputWidth();
    int height = det->getInputHeight();
    cv::Mat image;
    cv::resize(img, image, cv::Size(width, height), 0, 0,
               cv::INTER_LINEAR);
    auto result = det->run_8UC1(image);
    for (auto y = 0; y < result.segmentation.rows; y++) {
      for (auto x = 0; x < result.segmentation.cols; x++) {
            result.segmentation.at<uchar>(y,x) *= 10;
        }
    }
    cv::imwrite("segres.jpg",result.segmentation);

    auto resultshow = det->run_8UC3(image);
    resize(resultshow.segmentation, resultshow.segmentation, cv::Size(resultshow.cols * 2, resultshow.rows * 2));
    cv::imwrite("sample_segmentation_visualization_result.jpg",resultshow.segmentation);
   @endcode
 *
 * @image latex images/sample_segmentation_result.jpg "segmentation visulization result image" width=\textwidth
 *
 */

class Segmentation {
public:
  /**
     * @brief Factory function to get a instance of derived classes of class
     * Segmentation.
     *
     * Each Network has their own scale,such as FPN is 256 * 512;
     * @param Type, the tpye of segmentation network(FPN, ENET, ESPNET)
     * @param need_preprocess Normalize with mean/scale or not, default value is true.
     * @return An instance of segmentation class.
     *
     */
  static std::unique_ptr<Segmentation> create(Type type,
                                              bool need_preprocess = true);

protected:
  explicit Segmentation();
  Segmentation(const Segmentation &) = delete;

public:
  virtual ~Segmentation();

public:
  /**
   * @brief Function to get InputWidth of the segmentation network (input image cols).
   *
   * @return InputWidth of the segmentation network.
   */
  virtual int getInputWidth() const = 0;
 /**
   * @brief Function to get InputHight of the segmentation network (input image rows).
   *
   * @return InputHeight of the segmentation network.
   */
  virtual int getInputHeight() const = 0;
  /**
   * @brief Function of get running result of the segmentation network.
   *
   * @note The type of CV_8UC1 of the Reuslt's segmentation.
   *
   * @param img Input data of input image (cv::Mat).
   *
   * @return a result include segmentation output data.
   *
   */
  virtual SegmentationResult run_8UC1(const cv::Mat &image) = 0;

  /**
   * @brief Function of get running result of the segmentation network.
   *
   * @note The type of CV_8UC3 of the Reuslt's segmentation.
   * @param img Input data of input image (cv::Mat).
   *
   * @return a result include segmentation image and shape;.
   *
   */
  virtual SegmentationResult run_8UC3(const cv::Mat &image) = 0;
};
/**
 * @brief The Class of Segmentation8UC1, this class run function return a cv::Mat with the type is cv_8UC1
 * * sample code :
   @code
    auto det = xilinx::segmentation::Segmentation8UC1::create(xilinx::segmentation::FPN);
    auto img = cv::imread("sample_segmentation.jpg");

    int width = det->getInputWidth();
    int height = det->getInputHeight();
    cv::Mat image;
    cv::resize(img, image, cv::Size(width, height), 0, 0,
               cv::INTER_LINEAR);
    auto result = det->run(image);
    for (auto y = 0; y < result.segmentation.rows; y++) {
      for (auto x = 0; x < result.segmentation.cols; x++) {
            result.segmentation.at<uchar>(y,x) *= 10;
        }
    }
    cv::imwrite("segres.jpg",result.segmentation);
   @endcode
 *

 */
class Segmentation8UC1 {
public:
  /**
   * @brief Factory function to get a instance of derived classes of class
   * Segmentation8UC1.
   * @param Type, the tpye of segmentation network(FPN, ENET, ESPNET)
   * @param need_preprocess Normalize with mean/scale or not, default value
*is true.
   * @return An instance of segmentation8UC1 class.
   *
   */
  static std::unique_ptr<Segmentation8UC1> create(Type type,
                                                  bool need_preprocess = true) {
    return std::unique_ptr<Segmentation8UC1>(
        new Segmentation8UC1(Segmentation::create(type, need_preprocess)));
  }

protected:
  explicit Segmentation8UC1(std::unique_ptr<Segmentation> segmentation)
      : segmentation_{ std::move(segmentation) } {}
  Segmentation8UC1(const Segmentation8UC1 &) = delete;

public:
  ~Segmentation8UC1() {}

public:
  /**
    * @brief Function to get InputWidth of the segmentation network (input image
    *cols).
    *
    * @return InputWidth of the segmentation network.
    */
  virtual int getInputWidth() const { return segmentation_->getInputWidth(); }

  /**
    * @brief Function to get InputHight of the segmentation network (input image
    *cols).
    *
    * @return InputHeight of the segmentation network.
    */
  virtual int getInputHeight() const { return segmentation_->getInputHeight(); }

    /**
     *@brief Function of get running result of the segmentation network.
     *@note The result cv::Mat of the type is CV_8UC1.
     *@param image  Input data of the image (cv::Mat)
     *@return SegmentationResult The result of segmentation network.
     */
  virtual SegmentationResult run(const cv::Mat &image) {
    return segmentation_->run_8UC1(image);
  }

private:
  std::unique_ptr<Segmentation> segmentation_;
};


/**
 * @brief The Class of Segmentation8UC3, this class run function return a cv::Mat with the type is cv_8UC3
 * * sample code :
   @code
    auto det = xilinx::segmentation::Segmentation8UC3::create(xilinx::segmentation::FPN);
    auto img = cv::imread("sample_segmentation.jpg");

    int width = det->getInputWidth();
    int height = det->getInputHeight();
    cv::Mat image;
    cv::resize(img, image, cv::Size(width, height), 0, 0,
               cv::INTER_LINEAR);
    auto result = det->run(image);
    cv::imwrite("segres.jpg",result.segmentation);
   @endcode
 *
 */
class Segmentation8UC3 {
public:
  /**
  * @brief Factory function to get a instance of derived classes of class
  * Segmentation8UC3.
  * @param Type, the tpye of segmentation network(FPN, ENET, ESPNET)
  * @param need_preprocess Normalize with mean/scale or not, default value
*is true.
  * @return An instance of segmentation8UC3 class.
  *
  */
  static std::unique_ptr<Segmentation8UC3>
  create(Type type, bool need_preprocess = true) {
    return std::unique_ptr<Segmentation8UC3>(
        new Segmentation8UC3(Segmentation::create(type, need_preprocess)));
  }

protected:
  explicit Segmentation8UC3(std::unique_ptr<Segmentation> segmentation)
      : segmentation_{ std::move(segmentation) } {}
  Segmentation8UC3(const Segmentation8UC3 &) = delete;

public:
  ~Segmentation8UC3() {}

public:
  /**
  * @brief Function to get InputWidth of the segmentation network (input image
  *cols).
  *
  * @return InputWidth of the segmentation network.
  */
  virtual int getInputWidth() const { return segmentation_->getInputWidth(); }
  /**
     * @brief Function to get InputWidth of the segmentation network (input
    *image
     *cols).
     *
     * @return InputWidth of the segmentation network.
     */
  virtual int getInputHeight() const { return segmentation_->getInputHeight(); }
  /**
   *@brief Function of get running result of the segmentation network.
   *@note The result cv::Mat of the type is CV_8UC1.
   *@param image  Input data of the image (cv::Mat)
   *@return SegmentationResult The result of segmentation network.
   */
  virtual SegmentationResult run(const cv::Mat &image) {
      return segmentation_->run_8UC3(image);
  }

private:
  std::unique_ptr<Segmentation> segmentation_;
};
}
}
