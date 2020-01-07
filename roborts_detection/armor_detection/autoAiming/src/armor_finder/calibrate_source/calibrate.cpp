#include <armor_finder/armor_finder.h>

using namespace cv;

void ArmorFinder::initCalibrateParam() {
    std::string calibrate_intrinsic_filename = "~/urg_ws/src/RoboRTS/roborts_detection/armor_detection/autoAiming/extra_files/camera_calibration_parameter/intrinsics.yml";
    std::string calibrate_extrinsic_filename = "~/urg_ws/src/RoboRTS/roborts_detection/armor_detection/autoAiming/extra_files/camera_calibration_parameter/extrinsics.yml";
    Size image_size = Size(640, 480);
    Rect roi1, roi2;
    Mat Q;
    FileStorage fs_intrinsic(calibrate_intrinsic_filename, FileStorage::READ);
    FileStorage fs_extrinsic(calibrate_extrinsic_filename, FileStorage::READ);
    if(!fs_intrinsic.isOpened() || !fs_extrinsic.isOpened()){
        calibrate_param_.isAvailable = false;
        return;
    }
   Mat M1, D1, M2, D2;         //读取内参数矩阵文件中的矩阵
    fs_intrinsic["M1"]>>M1;
    fs_intrinsic["D1"]>>D1;
    fs_intrinsic["M2"]>>M2;
    fs_intrinsic["D2"]>>D2;  
    fs_intrinsic.release();

    Mat R, T, R1, P1, R2, P2;   //读取外参数矩阵文件中的矩阵
    fs_extrinsic["R"]>>R;
    fs_extrinsic["T"]>>T;    
    fs_extrinsic["R1"]>>R1;
    fs_extrinsic["R2"]>>R2;
    fs_extrinsic["P1"]>>P1;
    fs_extrinsic["P2"]>>P2;
    fs_extrinsic["Q"]>>Q;
    fs_extrinsic.release(); 

    initUndistortRectifyMap(M1, D1, R1, P1, image_size, CV_16SC2, calibrate_param_.map11, calibrate_param_.map12);
    initUndistortRectifyMap(M2, D2, R2, P2, image_size, CV_16SC2, calibrate_param_.map21, calibrate_param_.map22);
    calibrate_param_.isAvailable = true;
    std::cout<<"camera_calibration_parameter load succefully"<<std::endl;
}


void ArmorFinder::calibrate(cv::Mat &src_left, cv::Mat &src_right) {
    if(!calibrate_param_.isAvailable) return;

    if(src_left.empty() || src_right.empty())
        return;

    remap(src_left, src_left, calibrate_param_.map11, calibrate_param_.map12, INTER_LINEAR);
    remap(src_right, src_right, calibrate_param_.map21, calibrate_param_.map22, INTER_LINEAR);

}
