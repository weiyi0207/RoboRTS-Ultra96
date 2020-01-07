
#include "armor_finder/armor_finder.h"

using namespace cv;

void ArmorFinder::showTwoImages(std::string windows_name, const cv::Mat &src_left, const cv::Mat &src_right){
    static Mat image2show_left, image2show_right;

    if(src_left.type() == CV_8UC1) // 黑白图像
    {
        cvtColor(src_left, image2show_left, COLOR_GRAY2RGB);
        cvtColor(src_right, image2show_right, COLOR_GRAY2BGR);

    } else if (src_left.type() == CV_8UC3) //RGB 彩色
    {
        image2show_right = src_right.clone();
        image2show_left = src_left.clone();

    }
    Mat combined_image(image2show_left.rows, image2show_left.cols + image2show_right.cols, image2show_left.type());
    image2show_left.colRange(0, image2show_left.cols).copyTo(combined_image.colRange(0, image2show_left.cols));
    image2show_right.colRange(0, image2show_right.cols).copyTo(
            combined_image.colRange(image2show_left.cols, combined_image.cols));
    imshow(windows_name, combined_image);
}

void ArmorFinder::showArmorBoxVector(std::string windows_name,
                                     const cv::Mat &src_left, const vector<cv::Rect2d> &armor_box_left,
                                     const cv::Mat &src_right, const vector<cv::Rect2d> &armor_box_right) {
    static Mat image2show_left, image2show_right;
    if (src_left.type() == CV_8UC1) // 黑白图像
    {
        cvtColor(src_left, image2show_left, COLOR_GRAY2RGB);
        cvtColor(src_right, image2show_right, COLOR_GRAY2BGR);
    } else if(src_left.type() == CV_8UC3) //RGB 彩色
    {
        image2show_left = src_left.clone();
        image2show_right = src_right.clone();
    }
    Mat combined_image(image2show_left.rows, image2show_left.cols + image2show_right.cols, image2show_left.type());
    for (auto &a:armor_box_left) {
        rectangle(image2show_left, a, Scalar(0, 255, 0), 1);

    }
    for (auto &a:armor_box_right) {
        rectangle(image2show_right, a, Scalar(0, 255, 0), 1);
    }
    image2show_left.colRange(0, image2show_left.cols).copyTo(combined_image.colRange(0, image2show_left.cols));
    image2show_right.colRange(0, image2show_right.cols).copyTo(combined_image.colRange(image2show_left.cols, combined_image.cols));
    imshow(windows_name, combined_image);
}

void ArmorFinder::showArmorBox(std::string windows_name,
                                const cv::Mat &src_left, const cv::Rect2d &armor_box_left,
                                const cv::Mat &src_right, const cv::Rect2d &armor_box_right)
{
    static Mat image2show_left, image2show_right;
    if(src_left.type() == CV_8UC1) // 黑白图像
    {
        cvtColor(src_left, image2show_left, COLOR_GRAY2RGB);
        cvtColor(src_right, image2show_right, COLOR_GRAY2RGB);
    }
    else if(src_left.type() == CV_8UC3) //RGB 彩色
    {
        std::cout<<" is 8uc3"<<std::endl;
        image2show_left = src_left.clone();
        image2show_right = src_right.clone();
    }
    Mat combined_image(image2show_left.rows, image2show_left.cols+image2show_right.cols, image2show_left.type());
    rectangle(image2show_left, armor_box_left, Scalar(0, 255, 0), 3);
    rectangle(image2show_right, armor_box_right, Scalar(0, 255, 0), 3);
    image2show_left.colRange(0, image2show_left.cols).copyTo(combined_image.colRange(0, image2show_left.cols));
    image2show_right.colRange(0, image2show_right.cols).copyTo(combined_image.colRange(image2show_left.cols, combined_image.cols));
    imshow(windows_name, combined_image);

//    Rect box_on_raw_left = Rect2d(armor_box_left.x*2, armor_box_left.y*2, armor_box_left.width*2, armor_box_right.height*2);
//    Rect box_on_raw_right = Rect2d(armor_box_right.x*2, armor_box_right.y*2, armor_box_right.width*2, armor_box_right.height*2);
//
//    cvtColor(src_left_, image2show_left, COLOR_GRAY2RGB);
//    cvtColor(src_right_, image2show_right, COLOR_GRAY2RGB);
//
//    Mat combined_image2(image2show_left.rows, image2show_left.cols+image2show_right.cols, image2show_right.type());
//
//    rectangle(image2show_left, box_on_raw_left, Scalar(0, 255, 0), 1);
//    rectangle(image2show_right, box_on_raw_right, Scalar(0, 255, 0), 1);
//    image2show_left.colRange(0, image2show_left.cols).copyTo(combined_image2.colRange(0, image2show_left.cols));
//    image2show_right.colRange(0, image2show_right.cols).copyTo(combined_image2.colRange(image2show_left.cols, combined_image2.cols));
//
//    frame_to_display = combined_image2.clone();
//    putText(frame_to_display, std::to_string(armor_space_position_.z), Point(300, 100),cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(255, 200, 200), 1);
//    imshow("display ", frame_to_display);
    rectangle(image2show_left, armor_box_left, Scalar(0, 255, 0), 1);
    rectangle(image2show_right, armor_box_right, Scalar(0, 255, 0), 1);
    image2show_left.colRange(0, image2show_left.cols).copyTo(combined_image.colRange(0, image2show_left.cols));
    image2show_right.colRange(0, image2show_right.cols).copyTo(combined_image.colRange(image2show_left.cols, combined_image.cols));
    imshow(windows_name, combined_image);
}

void ArmorFinder::showContours(std::string windows_name,
                                const cv::Mat &src_left, const std::vector<LightBlob> &light_blobs_left,
                                const cv::Mat &src_right, const std::vector<LightBlob> &light_blobs_right)
{
    static Mat image2show_left, image2show_right;

    if(src_left.type() == CV_8UC1) // 黑白图像
    {
        cvtColor(src_left, image2show_left, COLOR_GRAY2RGB);
        cvtColor(src_right, image2show_right, COLOR_GRAY2BGR);
    }
    else if(src_left.type() == CV_8UC3) //RGB 彩色
    {
        image2show_left = src_left.clone();
        image2show_right = src_right.clone();
    }

    Mat combined_image(image2show_left.rows, image2show_left.cols+image2show_right.cols, image2show_left.type());
    for(const auto &light_blob:light_blobs_left)
    {
        rectangle(image2show_left, light_blob.rect.boundingRect(), Scalar(255,0,0), 3);
    }
    for(const auto &light_blob:light_blobs_right)
    {
        rectangle(image2show_right, light_blob.rect.boundingRect(), Scalar(255,0,0), 3);
    }
    image2show_left.colRange(0, image2show_left.cols).copyTo(combined_image.colRange(0, image2show_left.cols));
    image2show_right.colRange(0, image2show_right.cols).copyTo(combined_image.colRange(image2show_left.cols, combined_image.cols));
    imshow(windows_name, combined_image);


}


void ArmorFinder::showSpacePositionBackToStereoVision(const cv::Mat &src_left, const cv::Mat &src_right,
                                                      const cv::Point3d &space_position) {
    static Mat image2show_left, image2show_right;

    if(src_left.type() == CV_8UC1) // 黑白图像
    {
        cvtColor(src_left, image2show_left, COLOR_GRAY2RGB);
        cvtColor(src_right, image2show_right, COLOR_GRAY2BGR);
    }
    else if(src_left.type() == CV_8UC3) //RGB 彩色
    {
        image2show_left = src_left.clone();
        image2show_right = src_right.clone();
    }

    Mat combined_image(image2show_left.rows, image2show_left.cols+image2show_right.cols+10, image2show_left.type());

    Point2d left_center, right_center;
    left_center.x = space_position.x / space_position.z * stereo_camera_param_.FOCUS  / stereo_camera_param_.LENGTH_PER_PIXAL + 320;
    left_center.y = space_position.y / space_position.z * stereo_camera_param_.FOCUS  / stereo_camera_param_.LENGTH_PER_PIXAL + 240;

    double disparity = stereo_camera_param_.CAMERA_DISTANCE * stereo_camera_param_.FOCUS / space_position.z;
    right_center.x = left_center.x + disparity / stereo_camera_param_.LENGTH_PER_PIXAL;
    right_center.y = left_center.y;

    circle(image2show_left, left_center, 2, Scalar(0, 255, 0), 4);
    circle(image2show_right, right_center, 2, Scalar(0, 255, 0), 4);

    image2show_left.colRange(0, image2show_left.cols).copyTo(combined_image.colRange(0, image2show_left.cols));
    image2show_right.colRange(0, image2show_right.cols).copyTo(combined_image.colRange(image2show_left.cols+10, combined_image.cols));

    frame_to_display = src_left.clone();


    imshow("Reconstruct", combined_image);
}