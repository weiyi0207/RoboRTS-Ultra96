#include "armor_finder/armor_finder.h"

using namespace cv;
using namespace std;




bool ArmorFinder::stateSearchingTarget(cv::Mat &src_left_light, cv::Mat &src_right_light) {

    /************************** find light blobs **********************************************/

    imagePreprocess(src_left_light, src_right_light);  // bayer hacking, to split blue and red

    pipelineForFindLightBlob(src_left_light, src_right_light, light_blobs_left_real_, light_blobs_right_real_);

    /*************************** match light blobs***********************************/

    matchLightBlobVector(light_blobs_left_real_, armor_boxes_left_);
    //matchLightBlobVector(light_blobs_right_real_, armor_boxes_right_);
    //showArmorBoxVector("armor boxes", src_left_light, armor_boxes_left_, src_right_light, armor_boxes_right_);


    armor_boxes_right_.clear();
    if(armor_boxes_left_.empty()) return false;

    sort(armor_boxes_left_.begin(), armor_boxes_left_.end(),
         [](Rect2d a, Rect2d b) -> bool {return abs(a.x+a.width-340)+abs(a.y+a.height-280)<abs(b.x+b.width-340)+abs(b.y+b.height-280);});
    auto box_left = armor_boxes_left_.at(0);
    if( (Rect2d(0, 0, 640, 480)&box_left).area() < box_left.area()
        //|| (Rect2d(0, 0, 640, 480)&armor_box_right_).area() < armor_box_right_.area()
            ) // avoid box touching edges
    {
        return false;
    }

    Mat roi = src_raw_left_(box_left);
    Mat src_right = src_raw_right_(Rect2d(0, box_left.y, 640, box_left.height));
    Mat result;
    cv::matchTemplate(src_right, roi, result, cv::TM_SQDIFF);
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
    armor_box_left_ = box_left;
    armor_box_right_ = Rect2d(minLoc.x, box_left.y, box_left.width, box_left.height);

    return true;

//    bool state_match = matchTwoArmorBox(armor_boxes_left_, armor_boxes_right_, armor_box_left_, armor_box_right_);
//    if(!state_match) {
//	return false;}



//    /********************** convert to 3d coordinate *********************************/
//    convertToStereoscopicCoordinate(armor_box_left_, armor_box_right_, armor_space_position_);

//     //following lines are useful for aiming rotating target when the FPS is low
//     //it will skip some frame when target is changing rapidly
//    if(targetSearchPositionStreamControlWillSkip(armor_box_left_.x, armor_box_left_.y)){
//	sendTargetByUart(
//            static_cast<float>(armor_space_position_.x/3),
//            static_cast<float>(armor_space_position_.y/2),
//            static_cast<float>(armor_space_position_.z));
//
//
//        return false;
//    }

//    return computeAndSendAngle(armor_box_left_, armor_box_right_);
//
//    double x_diff_left = armor_box_left_.x + armor_box_left_.width/2- 320;
//    double y_diff_left = armor_box_left_.y + armor_box_left_.height/2 - 240;
//    double x_diff_right = armor_box_right_.x + armor_box_right_.width/2 - 320;
//    double y_diff_right = armor_box_right_.y + armor_box_right_.height/2 - 240;
//    double d1, d2, x_angle, y_angle, focus, distance;
//    d1 = stereo_camera_param_.CAMERA_DISTANCE * abs(x_diff_left) / (abs(x_diff_right) + abs(x_diff_left) +0.1);
//    d2 = stereo_camera_param_.CAMERA_DISTANCE - d1;
//    distance = d1 / (abs(x_diff_left) * stereo_camera_param_.LENGTH_PER_PIXAL ) * stereo_camera_param_.FOCUS;
//    x_angle = atan((d1 - stereo_camera_param_.CAMERA_DISTANCE/2) / distance) * 180 / 3.14;
//    y_angle = atan(y_diff_left * stereo_camera_param_.LENGTH_PER_PIXAL / stereo_camera_param_.FOCUS) * 180 / 3.14;
//
//    return sendTargetByUart(
//            static_cast<float>(x_angle),
//            static_cast<float>(y_angle),
//            static_cast<float>(distance));
}
