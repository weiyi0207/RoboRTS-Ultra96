#ifndef _XF_ACCEL_H_
#define _XF_ACCEL_H_

#include "xf_headers.h"
#include <vector>


//void cvMat2array(cv::Mat src, PIXEL* b, PIXEL* g, PIXEL* r);
//void dilate(cv::Mat& gray, cv::Mat& dilate);
//void bgr2gray(cv::Mat& bgr, cv::Mat& gray);
//void threshold(cv::Mat& gray, cv::Mat& binary, PIXEL threshold, PIXEL maxval);
std::vector<cv::RotatedRect> pre_process(const cv::Mat& bgr, PIXEL color_threshold, PIXEL bright_threshold, PIXEL maxval, int Enemycolor);



#endif
