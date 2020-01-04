#include <opencv2/opencv.hpp>
#include <string>

#include "proto/svm_predictor.pb.h"

class SVMPredictor
{
  public:
    SVMPredictor();

    void LordParam();

    bool Predict(cv::Mat patch);

  private:
    cv::Ptr<cv::ml::SVM> svm_model_;

    std::string model_name_;
    int resize_height_;
    int resize_width_;
};