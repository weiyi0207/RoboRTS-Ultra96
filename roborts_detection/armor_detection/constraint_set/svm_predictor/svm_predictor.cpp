#include "svm_predictor.h"

#include <string>
#include <iostream>
#include <fstream>

#include "io/io.h"

using namespace cv;
using namespace std;
using namespace cv::ml;


SVMPredictor::SVMPredictor()
{
    LordParam();
}

void SVMPredictor::LordParam() {
    SVMPredictorConfig svm_predictor_config;

    std::string file_name = ros::package::getPath("roborts_detection") + \
        "/armor_detection/constraint_set/svm_predictor/config/svm_predictor.prototxt";
    bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &svm_predictor_config);
    ROS_ASSERT_MSG(read_state, "Cannot open %s", file_name.c_str());

    model_name_ = ros::package::getPath("roborts_detection") + svm_predictor_config.model_name();
    resize_height_ = svm_predictor_config.resize_height();
    resize_width_ = svm_predictor_config.resize_width();

    svm_model_ = cv::ml::StatModel::load<cv::ml::SVM>(model_name_);
    if (!svm_model_.empty())
        ROS_INFO("Load model: %s", model_name_);
    else
        ROS_ERROR("Cannot load svm model %s", model_name_);
}

bool SVMPredictor::Predict(cv::Mat patch)
{
    Mat resize_image;
    vector<float> descriptors;
    HOGDescriptor hog(Size(32, 32), Size(16, 16), Size(8, 8), Size(8, 8), 9);
    resize(patch, resize_image, Size(resize_width_, resize_height_));
    hog.compute(resize_image, descriptors);
    float pre_result = svm_model_->predict(descriptors);
    return pre_result;
}