
#include <armor_finder/armor_finder.h>

#include "armor_finder/armor_finder.h"

using namespace cv;
using namespace std;


void ArmorFinder::initLightCoupleParam() {
    light_couple_param_.TWIN_ANGEL_MAX = 10;
    light_couple_param_.TWIN_LENGTH_RATIO_MAX = 4.0;
    light_couple_param_.TWIN_DISTANCE_N_MIN = 1.3;       // 1.7
    light_couple_param_.TWIN_DISTANCE_N_MAX = 3.8;       // 3.8
    light_couple_param_.TWIN_DISTANCE_T_MAX = 1.4;
    light_couple_param_.TWIN_AREA_RATIO_MAX = 3;
    light_couple_param_.TWIN_CENTER_POSITION_DIFF = 100;
}

void ArmorFinder::initArmorSeekingParam() {
    armor_seeking_param_.BORDER_IGNORE = 10;
    armor_seeking_param_.BOX_EXTRA = 5;
}

bool ArmorFinder::matchLightBlobVector(std::vector<LightBlob> &light_blobs, vector<cv::Rect2d> &armor_boxes) {
    armor_boxes.clear();
	

    if (light_blobs.size() < 2)
        return false;

    long light_index_left = -1;
    long light_index_right = -1;

    sort(light_blobs.begin(), light_blobs.end(),
         [](LightBlob a, LightBlob b) -> bool { return abs(a.rect.center.x-321)+abs(a.rect.center.y-300) < (b.rect.center.x-321)+abs(b.rect.center.y-300); });

    for (long i = 0; i < light_blobs.size() - 1; ++i) {
        for (long j = i + 1; j < light_blobs.size(); ++j) {
            if (!isCoupleLight(light_blobs.at(i), light_blobs.at(j))) {
                continue;
            }
            light_index_left = i;
            light_index_right = j;
            Rect2d rect_left = light_blobs.at(static_cast<unsigned long>(light_index_left)).rect.boundingRect();
            Rect2d rect_right = light_blobs.at(static_cast<unsigned long>(light_index_right)).rect.boundingRect();
            double min_x, min_y, max_x, max_y;
            min_x = min(rect_left.x, rect_right.x) - armor_seeking_param_.BOX_EXTRA;
            max_x = max(rect_left.x + rect_left.width, rect_right.x + rect_right.width) +
                    armor_seeking_param_.BOX_EXTRA;
            min_y = min(rect_left.y, rect_right.y) - armor_seeking_param_.BOX_EXTRA;
            max_y = max(rect_left.y + rect_left.height, rect_right.y + rect_right.height) +
                    armor_seeking_param_.BOX_EXTRA;
            if (min_x < 40 || max_x > 600 || min_y < 200 || max_y > 460) {
                continue;
            }
            armor_boxes.emplace_back(Rect2d(min_x, min_y, max_x - min_x, max_y - min_y));

        }

    }

    return light_index_left + light_index_right != -2;


}


double leastSquare(const LightBlob &light_blob) {
    double x_average = 0, y_average = 0, x_squa_average = 0, x_y_average = 0;
    for (auto &point:light_blob.contours) {
        x_average += point.x;
        y_average += point.y;
        x_squa_average += point.x * point.x;
        x_y_average += point.x * point.y;
    }
    x_average /= light_blob.contours.size();
    y_average /= light_blob.contours.size();
    x_squa_average /= light_blob.contours.size();
    x_y_average /= light_blob.contours.size();
    return (x_y_average - x_average * y_average) / (x_squa_average - x_average * x_average);

}

bool newAngelJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    return abs(atan(leastSquare(light_blob_i)) - atan(leastSquare(light_blob_j))) < 0.2;
}
bool oldAngelJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
//    Point2f side = light_blob_i.rect.center - light_blob_j.rect.center;
//    Point2f rect;
//    if (light_blob_i.rect.size.width >= light_blob_i.rect.size.height)
//        rect = Point2f(static_cast<float>(10 * cos(light_blob_i.rect.angle * 3.1415926 / 180)),
//                       static_cast<float>(10 * sin(light_blob_i.rect.angle * 3.1415926 / 180)));
//    else
//        rect = Point2f(static_cast<float>(10 * cos((light_blob_i.rect.angle + 90) * 3.1415926 / 180)),
//                       static_cast<float>(10 * sin((light_blob_i.rect.angle + 90) * 3.1415926 / 180)));
//    return abs(side.dot(rect) * side.dot(rect) / (side.dot(side) * rect.dot(rect))) < 0.03;
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
                    light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
                    light_blob_j.rect.angle - 90;
    return abs(angle_i-angle_j)<10;
}
bool heightJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
//    return abs(centers.y) / light_blob_i.length <= 2;
    return abs(centers.y)<25;
}

bool lengthJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    double side_length;
    Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
    side_length = sqrt(centers.ddot(centers));
    return (side_length / light_blob_i.length < 4 && side_length / light_blob_i.length > 1.5);
}

bool lengthRatioJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    return (light_blob_i.length / light_blob_j.length < 2
             && light_blob_i.length / light_blob_j.length > 0.5);
}

bool ArmorFinder::isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    return lengthRatioJudge(light_blob_i, light_blob_j) &&
            lengthJudge(light_blob_i, light_blob_j) &&
           heightJudge(light_blob_i, light_blob_j) &&
           oldAngelJudge(light_blob_i, light_blob_j);
//    return oldAngelJudge(light_blob_i, light_blob_j);
}


