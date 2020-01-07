#include "armor_finder/armor_finder.h"

using namespace cv;

void ArmorFinder::initTrackingParam(){
    track_param_.THRESHOLD_FOR_COUNT_NON_ZERO = 200;
    track_param_.TRANSFER_RATIO_OF_TRACKING_AREA_NONZERO = 0.5;

}


bool ArmorFinder::stateTrackingTarget(cv::Mat &src_left, cv::Mat &src_right) {

    /********************** tracking ***********************************************/
    track(kcf_tracker_left_, src_left, armor_box_left_);
    track(kcf_tracker_right_, src_right, armor_box_right_);
    auto box_left = armor_box_left_;
    if( (Rect2d(0, 0, 640, 480)&armor_box_left_).area() < armor_box_left_.area()
        || (Rect2d(0, 0, 640, 480)&armor_box_right_).area() < armor_box_right_.area()
            ) // avoid box touching edges
    {
        return false;
    }

    Mat roi = src_raw_left_(box_left);
    Mat src_raw_right = src_raw_right_(Rect2d(0, box_left.y, 640, box_left.height));
    Mat result;
    //cv::matchTemplate(src_raw_right, roi, result, cv::TM_SQDIFF);
    //double minVal, maxVal;
    //cv::Point minLoc, maxLoc;
    //cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
    //armor_box_right_ = Rect2d(minLoc.x, box_left.y, box_left.width, box_left.height);

    if( (Rect2d(0, 0, 640, 480)&armor_box_left_).area() < armor_box_left_.area()
        || (Rect2d(0, 0, 640, 480)&armor_box_right_).area() < armor_box_right_.area()
        ) // avoid box touching edges
    {
        return false;
    }

    Mat roi_left = src_left.clone()(armor_box_left_);
    Mat roi_right = src_right.clone()(armor_box_right_);
    threshold(roi_left, roi_left, track_param_.THRESHOLD_FOR_COUNT_NON_ZERO, 255, THRESH_BINARY);
    threshold(roi_right, roi_right, track_param_.THRESHOLD_FOR_COUNT_NON_ZERO, 255, THRESH_BINARY);
   // std::cout<<countNonZero(roi_left)<<" "<<total_contour_area_left_<<std::endl;
    if(abs(countNonZero(roi_left)-total_contour_area_left_) >= track_param_.TRANSFER_RATIO_OF_TRACKING_AREA_NONZERO * total_contour_area_left_
         || abs(countNonZero(roi_right)-total_contour_area_right_) >= track_param_.TRANSFER_RATIO_OF_TRACKING_AREA_NONZERO * total_contour_area_right_
        ){
            return false;
    }
    return true;

    //showArmorBox("tracking boxes", src_left, armor_box_left_, src_right, armor_box_right_);

//    /********************** convert to 3d coordinate *********************************/
//    convertToStereoscopicCoordinate(armor_box_left_, armor_box_right_, armor_space_position_);

    /*********************** compute angle **************************************************************/
//    return computeAndSendAngle(armor_box_left_, armor_box_right_);

//    double x_diff_left = armor_box_left_.x + armor_box_left_.width/2- 320;
//    double y_diff_left = armor_box_left_.y + armor_box_left_.height/2 - 240;
//    double x_diff_right = armor_box_right_.x + armor_box_right_.width/2 - 320;
//    double y_diff_right = armor_box_right_.y + armor_box_right_.height/2 - 240;
//    double d1, d2, x_angle, y_angle, focus, distance;
//    d1 = stereo_camera_param_.CAMERA_DISTANCE * abs(x_diff_left) / (abs(x_diff_right) + abs(x_diff_left));
//    d2 = stereo_camera_param_.CAMERA_DISTANCE - d1;
//    distance = d1 / (abs(x_diff_left) * stereo_camera_param_.LENGTH_PER_PIXAL ) * stereo_camera_param_.FOCUS;
//    x_angle = atan((d1 - stereo_camera_param_.CAMERA_DISTANCE/2) / distance) * 180 / 3.14;
//    y_angle = atan(y_diff_left * stereo_camera_param_.LENGTH_PER_PIXAL / stereo_camera_param_.FOCUS) * 180 / 3.14;
//
//
//
////    /*************** a predict function for moving target with only constant speed *******************/
////    targetTrackPositionStreamControl(armor_space_position_);
//
//    /********************** send it by uart and adjust the original point to the center *************/
//    return sendTargetByUart(
//            static_cast<float>(x_angle),
//            static_cast<float>(y_angle),
//            static_cast<float>(distance));
}
