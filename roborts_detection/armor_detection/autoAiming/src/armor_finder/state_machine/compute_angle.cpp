#include "armor_finder/armor_finder.h"

using namespace cv;
using namespace std;

bool ArmorFinder::computeAndSendAngle(cv::Rect2d &armor_box_left, cv::Rect2d &armor_box_right)
{

    double x_diff_left = armor_box_left.x + armor_box_left.width/2- 320;
    double y_diff_left = armor_box_left.y + armor_box_left.height/2 - 240;
    double x_diff_right = armor_box_right.x + armor_box_right.width/2 - 320;
    double y_diff_right = armor_box_right.y + armor_box_right.height/2 - 240;
    double d1, d2, x_angle, y_angle, focus, distance;
    d1 = stereo_camera_param_.CAMERA_DISTANCE * abs(x_diff_left) / (abs(x_diff_right) + abs(x_diff_left) +0.1);
    d2 = stereo_camera_param_.CAMERA_DISTANCE - d1;
    distance = d1 / (abs(x_diff_left) * stereo_camera_param_.LENGTH_PER_PIXAL ) * stereo_camera_param_.FOCUS;
    x_angle = atan((d1 - stereo_camera_param_.CAMERA_DISTANCE/2) / distance) * 180 / 3.14;
    y_angle = atan(y_diff_left * stereo_camera_param_.LENGTH_PER_PIXAL / stereo_camera_param_.FOCUS) * 180 / 3.14;
    
    return sendTargetByUart(
            static_cast<float>(x_angle),
            static_cast<float>(y_angle),
            static_cast<float>(distance));


}
