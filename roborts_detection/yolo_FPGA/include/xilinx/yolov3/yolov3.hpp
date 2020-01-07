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
  * Filename: yolov3.hpp
  *
  * Description:
  * This network is used to detecting object from a image, it will return
  * its coordinate, label and confidence.
  *
  * Please refer to document "Xilinx_AI_SDK_User_Guide.pdf" for more details
  * of these APIs.
  */
#ifndef DEEPHI_YOLO_V3_H_
#define DEEPHI_YOLO_V3_H_

#include <opencv2/core.hpp>
#include <memory>
#include <vector>

namespace xilinx {
namespace yolov3 {
/**
 *@struct YOLOv3Result
 *@brief Struct of the result returned by the yolov3 neuron network.
 *@note VOC dataset category:string label[20] = {"aeroplane", "bicycle", "bird",
 *"boat", "bottle", "bus","car", "cat", "chair", "cow", "diningtable", "dog",
 *"horse", "motorbike","person", "pottedplant", "sheep", "sofa", "train",
 *"tvmonitor"};
 *@note ADAS dataset category : string label[3] = {"car", "person", "cycle"};
 */
struct YOLOv3Result {
  /// Width of input image.
  int width;
  /// Height of output image.
  int height;
  /**
   *@struct BoundingBox
   *@Brief Struct of detection result with a object
   */
  struct BoundingBox {
    /// classification.
    int label;
    /// confidence, the range from 0 to 1.
    float score;
    /// x-coordinate, x is normalized relative to the input image cols, its
    /// value range from 0 to 1.
    float x;
    /// y-coordinate, y is normalized relative to the input image rows, its
    /// value range from 0 to 1.
    float y;
    /// width, width is normalized relative to the input image cols, its value
    /// from 0 to 1.
    float width;
    /// height, height is normalized relative to the input image rows, its value
    /// range from 0 to 1.
    float height;
  };
  ///all objects, The vector of BoundingBox .
  std::vector<BoundingBox> bboxes;
};

/**
 *@brief YOLOv3 network Type
 */
enum Type {
  /// VOC detect model,20 classes, input size is 416x416.
  VOC_416x416,
  /// ADAS vehicle detect model, 3 classes, input size is 512x256.
  ADAS_512x256,
  /// ADAS_vehicle detect model, 10 classes, no pruned.
  ADAS_512x288,
  /// no support yet, ADAS_vehicle detect model, tiny yolo.
  ADAS_TINY_416x416,

  ///  VOC detect model, train by tensorflow, input size is 416x416
  VOC_416x416_TF,

  NUM_OF_TYPE
};

/**
 * @brief Base class for detecting objects in a image (cv::Mat).
 *
 * Input is a image (cv::Mat).
 *
 * Output is position of the pedestrians in the input image.
 *
 * sample code:
 * @code
    auto yolo = xilinx::yolov3::YOLOv3::create(xilinx::yolov3::ADAS_512x256,
 true);
    Mat img = cv::imread("test.jpg");

    auto results = yolo->run(img);

    for(auto &box : results.bboxes){
      int label = box.label;
      float xmin = box.x * img.cols + 1;
      float ymin = box.y * img.rows + 1;
      float xmax = xmin + box.width * img.cols;
      float ymax = ymin + box.height * img.rows;
      if(xmin < 0.) xmin = 1.;
      if(ymin < 0.) ymin = 1.;
      if(xmax > img.cols) xmax = img.cols;
      if(ymax > img.rows) ymax = img.rows;
      float confidence = box.score;

      cout << "RESULT: " << label << "\t" << xmin << "\t" << ymin << "\t"
           << xmax << "\t" << ymax << "\t" << confidence << "\n";
      if (label == 0) {
        rectangle(img, Point(xmin, ymin), Point(xmax, ymax), Scalar(0, 255, 0),
                  1, 1, 0);
      } else if (label == 1) {
        rectangle(img, Point(xmin, ymin), Point(xmax, ymax), Scalar(255, 0, 0),
                  1, 1, 0);
      } else if (label == 2) {
        rectangle(img, Point(xmin, ymin), Point(xmax, ymax), Scalar(0, 0, 255),
                  1, 1, 0);
      } else if (label == 3) {
        rectangle(img, Point(xmin, ymin), Point(xmax, ymax),
                  Scalar(0, 255, 255), 1, 1, 0);
      }
    }
    imwrite("result.jpg", img);
   @endcode
 *
 * Display of the yolov3_ADAS_512x256 model results:
 * @image latex images/sample_yolov3_adas_512x256_result.jpg "out image" width=\textwidth
 */
class YOLOv3 {
public:
  /**
    * @brief Factory function to get a instance of derived classes of class
    * YOLOv3.
    *
    * @param type VOC_416x416 or ADAS_512x256
    *
    * @param need_mean_scale_process Normalize with mean/scale or not, default
    *value is true.
    *
    * @return An instance of YOLOv3 class.
    *
    */
  static std::unique_ptr<YOLOv3> create(Type type,
                                        bool need_mean_scale_process = true);

  /**
    * @brief Factory function to get a instance of derived classes of class
    * YOLOv3.
    * @note for internal use
    *
    * @param model_name
    *
    * @param need_mean_scale_process Normalize with mean/scale or not, default
    *value is true.
    *
    * @return An instance of YOLOv3 class.
    *
    */
   static std::unique_ptr<YOLOv3> create_ex(const std::string& model_name,
                                        bool need_mean_scale_process = true);

protected:
  explicit YOLOv3();
  YOLOv3(const YOLOv3 &) = delete;

public:
  virtual ~YOLOv3();

public:
  /**
   * @brief Function to get InputWidth of the YOLOv3 network (input image cols).
   *
   * @return InputWidth of the YOLOv3 network
   */
  virtual int getInputWidth() const = 0;
 /**
   *@brief Function to get InputHeigth of the YOLOv3 network (input image rows).
   *
   *@return InputHeight of the YOLOv3 network.
   */
  virtual int getInputHeight() const = 0;
  /**
   * @brief Function of get running result of the YOLOv3 neuron network.
   *
   * @param image Input data of input image (cv::Mat).
   *
   * @return YOLOv3Result.
   *
   */
  virtual YOLOv3Result run(const cv::Mat &image) = 0;
};
}
}

#endif
