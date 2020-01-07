#ifndef YOLO_DETECT_H
#define YOLO_DETECT_H
#include <xilinx/yolov3/yolov3.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <map>
#include <string>
#include <vector>
using namespace std;
using namespace cv;
vector<float> yolo_detect(Mat &img, int color_flag);
#endif
