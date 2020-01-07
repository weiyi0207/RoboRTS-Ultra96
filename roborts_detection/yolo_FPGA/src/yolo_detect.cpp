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
#include "yolo_detect.h"
/*
  This is an example on how to use customer-provided model for yolov3 lib.
  Below parameter of "yolov3_voc_416" in create_ex() is assumed as 
  the model provided by customer. 
  This parameter must be same as the kernel name in the configuration file 
  of prototxt in /etc/XILINX_AI_SDK.conf.d/.
  That means, the file name of the .prototxt, the kernel name, and the parameter
  must be same.  

  below is the detailed steps of HowTo.
    Note: 
      * replace the ... in below dir to your own directory
      * please set correct parameter when running below tool.
        You need refer to corresponding document for detailed information
        of the tool in SDK which mentioned below.

  1. prepare your own customer-provided model (maybe trained by caffe or tensorflow);
  2. convert your own model to Xilinx model format via convert tool provided by SDK.
  3. use dnnc tool to build your model into .elf file
     3.1. caffe model.    
      dnnc --prototxt=/home/.../yolov3_voc_416/deploy.prototxt 
        --caffemodel=/home/..../deploy.caffemodel --output_dir=/home/.... 
        --net_name=yolov3_voc_416 --dpu=4096FA --cpu_arch=arm64 --mode=normal
     3.2. tensorflow model
      dnnc --parser=tensorflow --frozen_pb=/home/.../deploy.pb --output_dir=/home/... 
        --net_name=ssd_your_kern_name --dpu=4096FA --cpu_arch=arm64 --mode=normal
  4. build your model into library. This step need cross-compiling tool.
      /home/.../aarch64-linux-gnu-g++ -nostdlib -fPIC -shared 
        /home/.../dpu_your_own_model.elf -o 
        libdpumodelssd_your_own_model.so
  5. place the built lib in /usr/lib or other library path which can be accessed.
  6. prepare your own prototxt file and place it in etc/XILINX_AI_SDK.conf.d/ .
     Please refer to the document on how to modify this file.

  test pic: use "sample_yolov3.jpg" for this test.

*/
auto yolo = xilinx::yolov3::YOLOv3::create_ex("yolo", true);       //argument true means yolo needs mean scale value in yolo.prototxt
vector<float> yolo_detect(Mat &img, int color_flags)
{
  if (img.empty()) {
    std::cout << "DPU didn't receive image"
              << std::endl;
  }
  vector <float> yolo_data;
  vector <float> yolo_undetected(5,-1);
  auto results = yolo->run(img);
  std::cout << "results.size " << results.bboxes.size() << " " //
            << std::endl;
  float conf;
  for(auto &box : results.bboxes){
 //     int label = box.label;
 //     float xmin = box.x * img.cols + 1;
 //     float ymin = box.y * img.rows + 1;
 //     float xmax = xmin + box.width * img.cols;
 //     float ymax = ymin + box.height * img.rows;
 //     if(xmin < 0.) xmin = 1.;
 //     if(ymin < 0.) ymin = 1.;
 //     if(xmax > img.cols) xmax = img.cols;
 //     if(ymax > img.rows) ymax = img.rows;
 //     float confidence = box.score;
      vector<float> yolo_tmp;
      cout << "label: " << box.label << " score:" << box.score << endl;
      if(box.label == color_flags) {
          yolo_tmp.push_back(box.x*img.cols+1);
          yolo_tmp.push_back(box.y*img.rows+1);
          yolo_tmp.push_back(box.width*img.cols);
          yolo_tmp.push_back(box.height*img.rows);
          yolo_tmp.push_back(box.score);
          conf = box.score;
          if(conf > 0.7f )    {
              yolo_data = yolo_tmp; 
              
              break;
          }
          else    yolo_data = yolo_undetected;  
      }


//      cout << "RESULT: " << label << "\t" << xmin << "\t" << ymin << "\t"
//           << xmax << "\t" << ymax << "\t" << confidence << "\n";
//      rectangle(img, Point(xmin, ymin), Point(xmax, ymax), Scalar(0, 255, 0),
//                  1, 1, 0);
  }

//  imwrite("sample_yolo_tiny.jpg", img);
  return yolo_data;
}
