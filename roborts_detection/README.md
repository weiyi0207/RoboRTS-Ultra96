# Robot Detection using Xilinx DPU

This folder contains the example software code to run robot detection with [Xilinx DPU](https://www.xilinx.com/products/intellectual-property/dpu.html#overview). 

**To learn more about Xilinx DPU, please visit:** [AI Developer Hub](https://www.xilinx.com/products/design-tools/ai-inference/ai-developer-hub.html#edge)

## Getting Started

- To boot the Ultra96 with Xilinx DPU, please refer to [RM-Ultra96-image](https://github.com/weiyi0207/RM-Ultra96-image/tree/RM_DPU)

- Before running the robot detction node with Xilinx DPU, users should first copy the Shared Libraries under *yolo_FPGA/* to */usr/lib*

- To run the robot detection node accelerated by Xilinx DPU, use the following command:

    `rosrun roborts_detection armor_detection_node`
    
    `rosrun roborts_detection armor_detection_client`
    
    `rosrun roborts_detection yolo_FPGA_node`
    
- To start enemy detection, users have to start action in armor_detection_client

## Directory structure

Below is the directory structure of the ROS workspace.
```
├── armor_detection                               // armor_detection_node
│   ├── armor_detection_algorithms.h
│   ├── armor_detection_base.h
│   ├── armor_detection_client.cpp
│   ├── armor_detection_node.cpp
│   ├── armor_detection_node.cpp.bp
│   ├── armor_detection_node.h
│   ├── armor_detection_node.h.bp
│   ├── autoAiming                                // contains camera configuration, tracker code, robot armor detection code(NOT USED)
│   │   ├── extra_files
│   │   │   └── camera_calibration_parameter
│   │   ├── include
│   │   │   ├── armor_finder
│   │   │   ├── camera
│   │   │   ├── tools
│   │   │   └── tracker
│   │   └── src
│   │       ├── armor_finder
│   │       ├── camera
│   │       ├── tools
│   │       └── tracker
│   ├── CMakeLists.txt
│   ├── config
│   │   └── armor_detection.prototxt
│   ├── constraint_set                   // contains original robot armor detection code provided by the RoboMaster official(NOT USED)
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   ├── constraint_set.cpp
│   │   ├── constraint_set.h
│   │   ├── proto
│   │   └── svm_predictor
│   ├── gimbal_control.cpp
│   ├── gimbal_control.h
│   └── proto
├── CMakeLists.txt
├── cmake_module
├── package.xml
├── util
│   ├── CMakeLists.txt
│   └── cv_toolbox.h
└── yolo_FPGA                          // yolo_FPGA_node, using Xilinx DPU to accelerate robot armor detection process
    ├── CMakeLists.txt
    ├── include
    │   ├── xilinx                     // Xilinx AI model Zoo, contains various models to adapt to different needs 
    │   │   ├── base
    │   │   ├── benchmark
    │   │   ├── classification
    │   │   ├── config
    │   │   ├── demo
    │   │   ├── facedetect
    │   │   ├── map
    │   │   ├── math
    │   │   ├── multitask
    │   │   ├── openpose
    │   │   ├── posedetect
    │   │   ├── refinedet
    │   │   ├── roadline
    │   │   ├── segmentation
    │   │   ├── ssd
    │   │   └── yolov3
    │   ├── yolo_detect.h
    │   └── yolo_messenger.h
    ├── libdpssd.so
    ├── libdpyolov3.so
    ├── libglog.so
    └── src
        ├── yolo_detect.cpp                // DPU yolo detection software code 
        └── yolo_messenger.cpp             // ROS node to subscribe or publish messages from or to other nodes
```
     
     
