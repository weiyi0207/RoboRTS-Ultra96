
#include "armor_finder/armor_finder.h"

using namespace cv;
using std::cout;
using std::endl;

int ArmorFinder::run(cv::Mat &src_left, cv::Mat &src_right) {
    src_raw_left_ = src_left.clone();
    src_raw_right_ = src_right.clone();
    if(src_raw_left_.type() == CV_8UC3)
    {
        cvtColor(src_raw_left_, src_raw_left_, CV_RGB2GRAY);
        cvtColor(src_raw_right_, src_raw_right_, CV_RGB2GRAY);
    }


    switch (cur_state_) {
        case SEARCHING_TARGET:
            target_detected = stateSearchingTarget(src_left, src_right);
            if (target_detected) {
                target_found_frame_cnt++;
            } else {
                target_found_frame_cnt = 0;
            }
            if (target_found_frame_cnt >=  1 ) {
                if ((Rect2d(0, 0, 640, 480) & armor_box_left_).area() < armor_box_left_.area()
                    || (Rect2d(0, 0, 640, 480) & armor_box_right_).area() < armor_box_right_.area()
                    ) { // avoid box touching edges
                    target_found_frame_cnt = 0;
                    break;
                }
                Mat roi_left = src_raw_left_.clone()(armor_box_left_);
                Mat roi_right = src_raw_right_.clone()(armor_box_right_);
                threshold(roi_left, roi_left, track_param_.THRESHOLD_FOR_COUNT_NON_ZERO, 255, THRESH_BINARY);
                threshold(roi_right, roi_right, track_param_.THRESHOLD_FOR_COUNT_NON_ZERO, 255, THRESH_BINARY);
                total_contour_area_left_ = countNonZero(roi_left);
                total_contour_area_right_ = countNonZero(roi_right);

                trackInit(kcf_tracker_left_, src_raw_left_, armor_box_left_);
               
                trackInit(kcf_tracker_right_, src_raw_right_, armor_box_right_);
                transferState(TRACKING_TARGET);
                std::cout<<"dive into tracking"<<std::endl;

            }
            break;

        case TRACKING_TARGET:
            target_detected=stateTrackingTarget(src_raw_left_, src_raw_right_);
            if (!target_detected) {
                target_unfound_frame_cnt++;
            } else{
                target_unfound_frame_cnt = 0;
            }
            if(target_unfound_frame_cnt >= 1){
                std::cout << "jump out tracking" << endl;
                transferState(SEARCHING_TARGET);
                target_unfound_frame_cnt = 0;
            }
            break;
        case STAND_BY:

            stateStandBy();
            transferState(SEARCHING_TARGET);
            break;
        default:
            cout << "incorrect state: " << cur_state_ << endl;
    }

    return 0;
}
