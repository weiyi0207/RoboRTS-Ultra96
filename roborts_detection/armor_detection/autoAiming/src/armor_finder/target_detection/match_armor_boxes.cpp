//
// Created by melonfish on 1/21/19.
//


#include <armor_finder/armor_finder.h>

#include "armor_finder/armor_finder.h"

using namespace cv;
using namespace std;

bool ArmorFinder::matchTwoArmorBox(vector<cv::Rect2d> &armor_box_list_left, vector<cv::Rect2d> &armor_box_list_right,
                                   cv::Rect2d &armor_box_left, cv::Rect2d &armor_box_right) {

	
    vector<cv::Rect2d> armor_box_candidate_left,armor_box_candidate_right;
    bool output = false;
    int count = 0;
    if (armor_box_list_left.empty() || armor_box_list_right.empty()) {
        return false;
    }
    for (int i = 0; i < armor_box_list_left.size(); ++i) {
        for (int j = 0; j < armor_box_list_right.size(); ++j) {

            if (abs(armor_box_list_left[i].width/armor_box_list_left[i].height-
            armor_box_list_right[i].width/armor_box_list_right[i].height) > 0.1){
                continue;
            }
            
            if (abs(armor_box_list_left[i].area() - armor_box_list_right[j].area()) < 300) {
                armor_box_left = armor_box_list_left[i];
                armor_box_right = armor_box_list_right[j];
                return true;
                //armor_box_candidate_left.emplace_back(armor_box_list_left[i]);
                //armor_box_candidate_right.emplace_back(armor_box_list_right[i]);

		//++count;
                //output=true;
            }
        }
    }
    //if(count>2) {
    //    if(armor_box_candidate_left[0].y<armor_box_candidate_left[1].y){
    //       armor_box_left = armor_box_candidate_left[0];
    //       armor_box_right = armor_box_candidate_right[0];
    //    }
    //    else{
    //       armor_box_left = armor_box_candidate_left[1];
    //       armor_box_right = armor_box_candidate_right[1];  
    //    }
    //}
    return false;
}
