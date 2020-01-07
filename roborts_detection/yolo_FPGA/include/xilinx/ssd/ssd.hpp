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
  * Filename: ssd.hpp
  *
  * Description:
  * This network is used to detecting objects from a input image.
  *
  * Please refer to document "xilinx_XILINX_AI_SDK_user_guide.pdf" for more details of
  * these APIs.
  */
#ifndef DEEPHI_SSD_H_
#define DEEPHI_SSD_H_

#include <opencv2/core.hpp>
#include <memory>

namespace xilinx {
namespace ssd {
/**
 *@struct SSDResult
 *@brief Struct of the result returned by the ssd neuron network.
 */
struct SSDResult {
    /// Width of input image.
    int width;
    /// Height of input image.
    int height;
    /**
     *@struct BoundingBox
     *@brief  Struct of a object coordinate ,confidence and classification.
     */
    struct BoundingBox{
      /// classification
      int label;
      /// confidence
      float score;
      /// x-coordinate, x is normalized relative to the input image cols ,the
      /// value range from 0 to 1.
      float x;
      /// y-coordinate ,y is normalized relative to the input image rows ,the
      /// value range from 0 to 1.
      float y;
      /// width, width is normalized relative to the input image cols ,the value
      /// range from 0 to 1.
      float width;
      /// height, height is normalized relative to the input image rows ,the
      /// value range from 0 to 1.
      float height;
    };
    /// all objects, a vector of BoundingBox
    std::vector<BoundingBox> bboxes;
};

enum Type {
  /// Adas vehicle model, input size is 480x360.
  ADAS_VEHICLE_V3_480x360,
  /// Video structurization model, input size is 480x360, it can fit 117g,
  /// 11.6g, 5.5g. This is 11.6g.
  TRAFFIC_480x360,
  /// Pedestrian detect, OPS is 5.9g. input size is 640x360, sight is adas.
  ADAS_PEDESTRIAN_640x360,

  /// Mobilenet ssd detection, 7 classes
  MOBILENET_480x360,
  /// Mobilenet_v2 ssd detection, 11 classes
  MOBILENET_V2_480x360,

  /// tensorflow ssd model:
  VOC_300x300_TF,

  NUM_OF_TYPE
};

/**
 * @brief Base class for detecting position of vehicle,pedestrian and so on.
 *
 * Input is a image (cv:Mat).
 *
 * Output is a vector<SSDResult>.
 *
 * sample code :
   @code
   Mat img = cv::imread("sample_ssd_TRAFFIC_480x360.jpg");
   auto ssd = xilinx::ssd::SSD::create(xilinx::ssd::TRAFFIC_480x360,true);
   auto results = ssd->run(img);
   for(const auto &r : results.bboxes){
      auto label = r.label;
      auto x = r.x * img.cols;
      auto y = r.y * img.rows;
      auto width = r.width * img.cols;
      auto heigth = r.height * img.rows;
      auto score = r.score;
      std::cout << "RESULT: " << label << "\t" << x << "\t" << y << "\t" << width
         << "\t" << height << "\t" << score << std::endl;
   }
   @endcode
 *
 * Display of the ssd_TRAFFIC_480x360 model results:
 * @image latex images/sample_ssd_TRAFFIC_480x360_result.jpg "out image" width=\textwidth
 *
 * Display of the ADAS_VEHICLE_V3_480x360 model results:
 * @image latex images/sample_ssd_ADAS_VEHICLE_V3_480x360_result.jpg "out image" width=\textwidth
 */
class SSD {
public:
    /**
     * @brief Factory function to get a instance of derived classes of class
     * SSD.
     *
     * @param type Enum Type
     * @param @param need_mean_scale_process Normalize with mean/scale or not, default value is true.
     * @return An instance of SSD class.
     *
     */
  static std::unique_ptr<SSD> create(Type type,
                                     bool need_mean_scale_process = true);
   /**
     * @brief Factory function to get a instance of derived classes of class
     * SSD.
     *
     * @note for internal use
     * @param model_name String of model name
     * @param @param need_mean_scale_process Normalize with mean/scale or not, default value is true.
     * @return An instance of SSD class.
     *
     */
  static std::unique_ptr<SSD> create_ex(const std::string& model_name,
                                     bool need_mean_scale_process = true);

protected:
  explicit SSD();
  SSD(const SSD &) = delete;

public:
  virtual ~SSD();

public:
  /**
   * @brief Function to get InputWidth of the SSD network (input image cols).
   *
   * @return InputWidth of the SSD network.
   */
    virtual int getInputWidth() const = 0;
   /**
   *@brief Function to get InputHeigth of the SSD network (input image rows).
   *
   *@return InputHeight of the SSD network.
   */

    virtual int getInputHeight() const = 0;
 /**
   * @brief Function of get running result of the ssd neuron network.
   *
   * @param img Input data of input image (cv::Mat).
   *
   * @return SSDResult.
   *
   */
  virtual SSDResult run(const cv::Mat &img) = 0;
};
}
}

#endif
