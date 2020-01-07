//
// Created by zhikun on 18-12-2.
//

#ifndef STEREOVISION_PARAM_DEFINE_H
#define STEREOVISION_PARAM_DEFINE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

struct LightBlob {
    cv::RotatedRect rect;
    std::vector<cv::Point> contours;
    double length;

    explicit LightBlob(std::vector<cv::Point> &c) : contours(c) {
        rect = minAreaRect(c);
        length = std::max(rect.size.height, rect.size.width);
    };
    bool operator<(LightBlob &l2) { return this->rect.center.x < l2.rect.center.x; }
    bool operator<=(LightBlob &l2) { return this->rect.center.x <= l2.rect.center.x; }
    bool operator>(LightBlob &l2) { return this->rect.center.x > l2.rect.center.x; }
    bool operator>=(LightBlob &l2) { return this->rect.center.x >= l2.rect.center.x; }
};

struct LightBlobParam {
    int PREPROCESS_SUBSTRACT_FACTOR;
    float PREPROCESS_MULTIPLY_FACTOR;

    int GRAY_THRESH;
    long CONTOUR_AREA_MAX;
    long CONTOUR_AREA_MIN;
    long CONTOUR_LENGTH_MIN;
    float CONTOUR_HW_RATIO_MAX;
    float CONTOUR_HW_RATIO_MIN;
    float CONTOUR_ANGLE_MAX;
    float Y_POSITION_MIN;
};

struct LightCoupleParam{
    float TWIN_ANGEL_MAX;
    float TWIN_LENGTH_RATIO_MAX;
    float TWIN_DISTANCE_N_MIN;
    float TWIN_DISTANCE_N_MAX;
    float TWIN_DISTANCE_T_MAX;
    float TWIN_AREA_RATIO_MAX;
    float TWIN_CENTER_POSITION_DIFF;
};

struct StereoCameraPara {
    double CAMERA_DISTANCE;
    double FOCUS;
    double LENGTH_PER_PIXAL;
    int WIDTH, HEIGHT;
    double POSITION_INRTIA_RATIO;
};

struct ArmorSeekingParam{
    int BORDER_IGNORE;
    int BOX_EXTRA;

};

struct ArmorPridictParam{
    int ARMOR_POSITION_HISTORY_MAX_LENGTH;
    clock_t time_predict;
};


enum StateMachine{
    STAND_BY,
    SEARCHING_TARGET,
    TRACKING_TARGET
};

struct StateMachineParam{
    int THRESHOLD_FOUND_SEARCHING_TO_TRACKING;
    int THRESHOLD_UNFOUND_TRACKING_TO_SEARCHING;
};

struct CalibrateParam{
    bool isAvailable;
    cv::Mat map11, map12, map21, map22;
};

struct TrackingParam{
    int THRESHOLD_FOR_COUNT_NON_ZERO;
    double TRANSFER_RATIO_OF_TRACKING_AREA_NONZERO;
};


#endif //STEREOVISION_PARAM_DEFINE_H
