/**
 * this file contains a tool to calibrate stereo cameras
 *
 * usage:
 *
 * Mat src_left, src_right;
 * while(true)
 * {
 *      //get a frame of src_left and src_right
 *      if(tool_for_calibrate(src_left, src_right))
 *      {
 *          waitKey(0);
 *          break;
 *      }
 * }
 *
 *
 * output parameter file should be in "../extra_file/camera_calibration_parameter/"
 */

#ifndef SJTU_RM2019_WINTER_VERSION_CALIBRATE_TOOL_H
#define SJTU_RM2019_WINTER_VERSION_CALIBRATE_TOOL_H

#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/**
 * @brief a tool for calibrate, but it may not work properly now
 * @param src_left
 * @param src_right
 * @return
 */
bool tool_for_calibrate(const cv::Mat &src_left, const cv::Mat &src_right);


#endif //SJTU_RM2019_WINTER_VERSION_CALIBRATE_TOOL_H
