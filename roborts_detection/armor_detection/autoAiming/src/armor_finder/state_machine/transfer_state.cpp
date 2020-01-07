#include "armor_finder/armor_finder.h"

using namespace cv;


void ArmorFinder::initStateMachineParam() {
    state_machine_param_.THRESHOLD_FOUND_SEARCHING_TO_TRACKING = 1;
    state_machine_param_.THRESHOLD_UNFOUND_TRACKING_TO_SEARCHING = 1;

}

void ArmorFinder::transferState(StateMachine state){
    cur_state_ = state;
    target_found_frame_cnt = 0;
    target_unfound_frame_cnt = 0;

}
