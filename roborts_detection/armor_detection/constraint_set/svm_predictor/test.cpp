#include "svm_predictor.h"

int main(int argv, char* args[]) {
    std::cout << args[1] << std::endl;
    cv::Mat im = cv::imread(args[1]);
    std::cout << (im.empty()) << std::endl;
    std::cout << im.size() << std::endl;
    SVMPredictor svm;
    std::cout << "[[[[[[[[[[[" << svm.Predict(im) << "]]]]]]]]]]]]]\n";
    return 0;
}