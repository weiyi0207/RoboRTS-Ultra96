#include "armor_finder/armor_finder.h"

using namespace cv;




void ArmorFinder::trackInit(KCFTracker &kcf_tracker, cv::Mat &src, cv::Rect2d &armor_box) {
    kcf_tracker.init(armor_box, src);
}


bool ArmorFinder::track(KCFTracker &kcf_tracker, cv::Mat &src, cv::Rect2d &armor_box){
    bool ok = true;
    int BORDER_IGNORE = 10;
    armor_box = kcf_tracker.update(src);
    if(armor_box.x < BORDER_IGNORE ||armor_box.y < BORDER_IGNORE ||
    armor_box.x + armor_box.width > stereo_camera_param_.WIDTH - BORDER_IGNORE ||
    armor_box.y + armor_box.height > stereo_camera_param_.HEIGHT - BORDER_IGNORE){
        ok = false;
    }
    return ok;
}