//
// Created by zhikun on 18-11-11.
// this file used for testing detecting armor by light blob
// testing armor is large armor
//

#ifndef STEREOVISION_FROM_VIDEO_FILE_ARMOR_FINDER_H
#define STEREOVISION_FROM_VIDEO_FILE_ARMOR_FINDER_H

#include <algorithm>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <armor_finder/param_struct_define.h>
#include "armor_finder/constant.h"

#include <tracker/kcftracker.hpp>
#include "tracker/tracker.h"


using std::vector;

/**
 * @brief   a class to find armor in image. SJTU-TPP@RoboMaster2019
 */
class ArmorFinder {
public:
    ArmorFinder();

    ~ArmorFinder()= default;
    cv::Mat src_left_, src_right_;
    cv::Mat src_blue0, src_red0, src_blue1, src_red1;
    cv::Mat src_raw_right_, src_raw_left_;
    cv::Mat src_bin_left_, src_bin_right_;

    /**
     * @brief the interface of armor_finder
     * @param src_left : input
     * @param src_right : input
     * @return : bool value: whether it success.
     */
    int run(cv::Mat &src_left, cv::Mat &src_right);

public:
    // for debug or recording
    cv::Mat frame_to_display;

    /**
     * parameter structures, defined in param_struct_define.h
     */
    LightBlobParam light_blob_param_;
    LightCoupleParam light_couple_param_;
    StereoCameraPara stereo_camera_param_;
    ArmorSeekingParam armor_seeking_param_;
    ArmorPridictParam armor_predict_param_;
    StateMachineParam state_machine_param_;
    CalibrateParam calibrate_param_;
    TrackingParam track_param_;

    /**
     * vectors to store light blobs
     */
    std::vector<LightBlob> light_blobs_left_light_, light_blobs_right_light_;
    std::vector<LightBlob> light_blobs_left_color_, light_blobs_right_color_;
    std::vector<LightBlob> light_blobs_left_real_, light_blobs_right_real_;
    std::vector<int> armor_num_left, armor_num_right;

public:
    /**
     * Rects to store the found armor box position
     */
    cv::Rect2d armor_box_left_, armor_box_right_;

private:
    /**
     * Rects list to store all the possible armor box position
     */
    std::vector<cv::Rect2d> armor_boxes_left_, armor_boxes_right_;

    /**
     * a counter for changing state from searching to tracking
     */
    int target_found_frame_cnt, target_unfound_frame_cnt;

    /**
     * Finite state machine that switch between searching and tracking
     */
    StateMachine cur_state_;

    /**
     * Wrapped class to send message to lower computer
     */
   

    /**
     * A well functioned class for tracking
     */
    KCFTracker kcf_tracker_left_, kcf_tracker_right_;

    /**
     * enemy color define by two constant, defined in constant.h
     */
    int enemy_color_;

    /**
     * variable to determine the exit condition of tracking
     */
    double total_contour_area_right_;
    double total_contour_area_left_;

    /**
     * about armor space positions
     */
    cv::Point3d armor_space_position_;
    cv::Point3d armor_space_last_position_;
    std::vector<cv::Point3d> armor_history_positions_;
    cv::Point3d armor_predicted_position_;
public:
    bool target_detected;
    /**
     * variable to store the position difference between current and last
     */
    double position_diff = 0;


public:
    void setEnemyColor(int color);

    /**
     * @brief calibrate the camera, the parameter files should be in extra_files/camera_calibration_parameter/
     * @param src_left : inoutput
     * @param src_right : inoutput
     */
    void calibrate(cv::Mat &src_left, cv::Mat &src_right);

private:
    /**
     * initialize parameters, the definition should be in their related module respectively
     */
    void initCalibrateParam();
    void initLightParam();
    void initLightCoupleParam();
    void initCameraParam();
    void initArmorSeekingParam();
    void initArmorPredictParam();
    void initUartParam();
    void initStateMachineParam();
    void initTrackingParam();

    /**
     * @brief transfer FSM between searching and tracking, and reset the recording variables.
     * @param state
     */
    void transferState(StateMachine state);

    /**
     * @brief a not used state
     * @return
     */
    bool stateStandBy();

    /**
     * @brief searching state, it will search the entire frame to try to find a target
     * @param src_left
     * @param src_right
     * @return
     */
    bool stateSearchingTarget(cv::Mat &src_left, cv::Mat &src_right);

    /**
     * @brief tracking state, it will tracking the given area until the condition is not met
     * @param src_left
     * @param src_right
     * @return
     */
    bool stateTrackingTarget(cv::Mat &src_left, cv::Mat &src_right);

    /**
     * @brief split the unprocessed bayer matrix into blue and red. The blue and red are only 1/4 of the raw image
     * @param src : raw bayer matrix 640*480
     * @param blue : blue part 320*240
     * @param red : red part 320*240
     */
    void splitBayerBG(cv::Mat &src, cv::Mat &blue, cv::Mat &red);

    /**
     * @brief some preprocess of image, make image from camera and video file the same.
     * @param src_left
     * @param src_right
     */
    void imagePreprocess(cv::Mat &src_left, cv::Mat &src_right);

    /**
     * @brief find light blobs from the images.
     * @param src_left :image
     * @param src_right :image
     * @param light_blobs_real_left
     * @param light_blobs_real_right
     * @return
     */
    bool pipelineForFindLightBlob(cv::Mat &src_left, cv::Mat &src_right, std::vector<LightBlob> &light_blobs_real_left, std::vector<LightBlob> &light_blobs_real_right);

    /**
     * @brief process for find light blobs, enlarge the difference between the dark and light, to highlight the light blob
     * @param InOutput
     */
    void pipelineLightBlobPreprocess(cv::Mat &InOutput);

    /**
     * @brief untested function, to recoginize the digit on the armor
     * @param image
     * @return
     */
    int recognize_digits(cv::Mat &image);

    /**
     * @brief control the message stream to lower computer in searching state, some large jump will be skipped
     * @param x
     * @param y
     * @return
     */
    bool targetSearchPositionStreamControlWillSkip(double x, double y);

    /**
     * @brief control the message stream to lower computer in tracking state, we can do some prediction in tracking,
     *          because in tracking state the target almost moves smoothly
     * @param armor_position
     * @return
     */
    bool targetTrackPositionStreamControl(cv::Point3d &armor_position);

public:
    /**
     * @brief When there are many possible armors, it matches the most possible pair
     * @param armor_box_list_left
     * @param armor_box_list_right
     * @param armor_box_left
     * @param armor_box_right
     * @return
     */
    bool matchTwoArmorBox(vector<cv::Rect2d> &armor_box_list_left, vector<cv::Rect2d> &armor_box_list_right,
            cv::Rect2d &armor_box_left, cv::Rect2d &armor_box_right);

public:

    /**
     * a function to clear all vector for searching
     */
    void clear_light_blobs_vector();


public:
    /**
     * @name    findLightBlob()
     * @brief   Find out all the light blobs in given image
     * @param   src: input image
     * @param   light_blobs: output vector of light blobs
     * @return  bool value: whether it finds more than 2 valid light blobs
     */
    bool findLightBlob(const cv::Mat &src, std::vector<LightBlob> &light_blobs);

public:

    /**
     * @brief match light blobs to find a possible armor
     * @param light_blobs
     * @param armor_box
     * @return
     */
    bool matchLightBlobVector(std::vector<LightBlob> &light_blobs, vector<cv::Rect2d> &armor_box);

public:
    /**
     * @brief   convert two 2D coordinates to 3D coordinate
     * @param   armor_box_left: input armor box rect in left camera
     * @param   armor_box_right: input armor box rect in right camera
     * @param   space_position: output 3D point of armor
     * @return  bool value: whether it can be converted
     */
    bool convertToStereoscopicCoordinate(
            cv::Rect2d &armor_box_left, cv::Rect2d &armor_box_right, cv::Point3d &space_position);

public:
    /**
     * @brief still untested
     * @param armor_history_position
     * @param armor_predicted_position
     * @return
     */
    bool predictArmorPosition(
            cv::Point3d &armor_history_position, cv::Point3d &armor_predicted_position);

private:
    bool computeAndSendAngle(cv::Rect2d &armor_box_left, cv::Rect2d &armor_box_right);

public:
    /**
     * @brief send target position to lower computer by uart
     * @param x
     * @param y
     * @param z
     * @return
     */
    bool sendTargetByUart(float x, float y, float z);

private:
    /**
     * @name    isValidLightContour()
     * @brief   judge a contour whether valid or not
     * @param   light_contour: input light contour
     * @return  bool value: whether the light contour is valid
     */
    bool isValidLightContour(const vector<cv::Point> &light_contour);


    /**
     * @name    isCoupleLight()
     * @brief   judge two light blobs are a couple or not
     * @param   light_blob_x: input light blob x
     * @param   light_blob_y: input light blob y
     * @return  bool value: whether the two light blob is a couple
     */
    bool isCoupleLight(const LightBlob &light_blob_x, const LightBlob &light_blob_y);


    void manageHistorySpacePosition(const cv::Point3d &space_position);

public:
    /**
     * @brief all those show*** functions are used to display the images with found light blobs or armor rect.
     * @param windows_name
     * @param src0
     * @param src1
     */
    void showTwoImages(std::string windows_name, const cv::Mat &src0, const cv::Mat &src1);

    void showContours(std::string windows_name, const cv::Mat &src_left, const std::vector<LightBlob> &light_blobs_left,
            const cv::Mat &src_right, const std::vector<LightBlob> &light_blobs_right);


    void showArmorBox(std::string windows_name, const cv::Mat &src_left, const cv::Rect2d &armor_box_left,
            const cv::Mat &src_right, const cv::Rect2d &armor_box_right);

    void showArmorBoxVector(std::string windows_name, const cv::Mat &src_left, const vector<cv::Rect2d> &armor_box_left,
                            const cv::Mat &src_right, const vector<cv::Rect2d> &armor_box_right);

    void showSpacePositionBackToStereoVision(
            const cv::Mat &src_left, const cv::Mat &src_right, const cv::Point3d &space_position);


    /**
     * @brief give the tracker an area to let it track
     * @param kcf_tracker
     * @param src
     * @param armor_box
     */
    void trackInit(KCFTracker &kcf_tracker, cv::Mat &src, cv::Rect2d &armor_box);

    /**
     * @brief the tracker will give where the area is in given image
     * @param kcf_tracker
     * @param src
     * @param armor_box
     * @return
     */
    bool track(KCFTracker &kcf_tracker, cv::Mat &src, cv::Rect2d &armor_box);

};


#endif //STEREOVISION_FROM_VIDEO_FILE_ARMOR_FINDER_H
