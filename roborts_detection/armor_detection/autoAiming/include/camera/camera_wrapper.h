//
// Created by zhikun on 18-11-7.
// used for testing double cameras
// camera0 is left camera, camera1 is right camera.
//

#ifndef VIDEO_TEST1_CAMERA_WRAPPER_H
#define VIDEO_TEST1_CAMERA_WRAPPER_H

#include <stdio.h>
#include <iostream>
#include <thread>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include "wrapper_head.h"
#include "camera_api.h"

class CameraWrapper: public WrapperHead {
private:
    unsigned char* rgb_buffer0;
    unsigned char* rgb_buffer1;
    int camera_cnts;
    int camera_status0, camera_status1;
    tSdkCameraDevInfo camera_enum_list[2];
    int h_camera0;
    int h_camera1;
    char camera_name0[32];
    char camera_name1[32];

    tSdkCameraCapbility tCapability0;
    tSdkCameraCapbility tCapability1;
    tSdkFrameHead frame_info0;
    tSdkFrameHead frame_info1;
    BYTE *pby_buffer0;
    BYTE *pby_buffer1;
    IplImage* iplImage0;
    IplImage* iplImage1;
    int channel0;
    int channel1;
    bool read_state0, read_state1;



    void swapCameraHandle();

public:
    CameraWrapper();
    ~CameraWrapper() final;

    /**
     * @brief initialize the cameras, including connecting devices, setting handle and so on
     * @return
     */
    bool init() final;

    /**
     * @brief read image from cameras,
     * @param src0
     * @param src1
     * @return
     */
    bool read(cv::Mat& src0, cv::Mat& src1) final;

    /**
     * @brief read the image without process, it is a single channel, but it is a bayer matrix
     * @param src0
     * @param src1
     * @return
     */
    bool readRaw(cv::Mat& src0, cv::Mat& src1);

    /**
     * @brief read the image with process, it is three channels color image, but it is slower.
     * @param src0
     * @param src1
     * @return
     */
    bool readProcessed(cv::Mat& src0, cv::Mat& src1);

    /**
     * @brief try to read the camera image in two thread, (but it seems that the reading is already implemented in thread.)
     * @param src0
     * @param src1
     * @return
     */
    bool read_thread(cv::Mat& src0, cv::Mat& src1);
    void read_camera_thread0(cv::Mat &src);
    void read_camera_thread1(cv::Mat &src);

};


#endif //VIDEO_TEST1_CAMERA_WRAPPER_H
