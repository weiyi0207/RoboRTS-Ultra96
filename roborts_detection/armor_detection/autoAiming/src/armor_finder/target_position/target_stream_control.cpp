#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "armor_finder/armor_finder.h"

#include <iostream>


bool ArmorFinder::targetSearchPositionStreamControlWillSkip(double x, double y) {
    static double last_x = 0;
    double cur_diff = abs(x - last_x);
    bool willSikp = cur_diff > position_diff*1.5;

    double alpha = 0.8;
    position_diff =  alpha * cur_diff + (1-alpha) * position_diff ;
    last_x = x;

    return willSikp;
}


bool ArmorFinder::targetTrackPositionStreamControl(cv::Point3d &armor_position){
    static double last_x = 0;
    double cur_diff = abs(armor_position.x - last_x);
    if(cur_diff <= 0.5 && abs(armor_position.x) > 5 ){
        //armor_position.x *= 2;
        //std::cout<<" double position, abs of x "<<abs(armor_position.x)<<std::endl;
    }
    if(abs(armor_position.x) < 0.5)
        armor_position.x /= 5;

    double alpha = 0.8;
    position_diff = alpha * cur_diff + (1-alpha) * position_diff;
    double beta = 0.8;
    armor_position.x = beta * armor_position.x + (1 - beta) * last_x;
    last_x = armor_position.x;
    return true;

}
