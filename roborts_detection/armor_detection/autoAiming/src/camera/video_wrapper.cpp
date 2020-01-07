//
// Created by zhikun on 18-11-16.
//

#include "camera/video_wrapper.h"


VideoWrapper::VideoWrapper(const std::string &filename0, const std::string &filename1) {
    video0.open(filename0);
    //video1.open(filename1);
}

VideoWrapper::~VideoWrapper() = default;


bool VideoWrapper::init() {
    return video0.isOpened() && video0.isOpened();
}

bool VideoWrapper::read(cv::Mat &src_left, cv::Mat &src_right) {
    return video0.read(src_left) && video0.read(src_right);




}
