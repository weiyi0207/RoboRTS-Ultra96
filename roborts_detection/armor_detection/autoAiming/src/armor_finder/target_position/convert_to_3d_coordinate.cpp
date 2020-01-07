
#include "armor_finder/armor_finder.h"
#include "armor_finder/constant.h"

using namespace cv;
using std::cout;
using std::endl;

void ArmorFinder::initCameraParam() {
    stereo_camera_param_.CAMERA_DISTANCE = 19;  // cm
    stereo_camera_param_.FOCUS = 0.36;          // cm
    stereo_camera_param_.LENGTH_PER_PIXAL = 0.48/640;

}

//获取窗口内子图像
bool getSubImg(cv::Mat srcImg, cv::Point2f currPoint, cv::Mat &subImg)
{
    int subH = subImg.rows;
    int subW = subImg.cols;
    int x = int(currPoint.x+0.5f);
    int y = int(currPoint.y+0.5f);
    int initx = x - subImg.cols / 2;
    int inity = y - subImg.rows / 2;
    if (initx < 0 || inity < 0 || (initx+subW)>=srcImg.cols || (inity+subH)>=srcImg.rows )   return false;
    cv::Rect imgROI(initx, inity, subW, subH);
    subImg = srcImg(imgROI).clone();
    return true;
}

//亚像素角点提取
void myCornerSubPix(cv::Mat &srcImg, cv::Point2d &currPoint, cv::Size winSize, cv::Size zeroZone, cv::TermCriteria &criteria)
{
    //搜索窗口大小
    int winH = winSize.width * 2 + 1;
    int winW = winSize.height * 2 + 1;
//    int winCnt = winH*winW;

    //迭代阈值限制
    int MAX_ITERS = 100;
    int max_iters = (criteria.type & CV_TERMCRIT_ITER) ? MIN(MAX(criteria.maxCount, 1), MAX_ITERS) : MAX_ITERS;
    double eps = (criteria.type & CV_TERMCRIT_EPS) ? MAX(criteria.epsilon, 0.) : 0;
    eps *= eps; // use square of error in comparsion operations

    //生成高斯权重
    cv::Mat weightMask = cv::Mat(winH, winW, CV_32FC1);
    for (int i = 0; i < winH; i++)
    {
        for (int j = 0; j < winW; j++)
        {
            float wx = (float)(j - winSize.width) / winSize.width;
            float wy = (float)(i - winSize.height) / winSize.height;
            float vx = exp(-wx*wx);
            float vy = exp(-wy*wy);
            weightMask.at<float>(i, j) = (float)(vx*vy);
        }
    }

    double a, b, c, bb1, bb2;

    cv::Mat subImg = cv::Mat::zeros(winH+2, winW+2, CV_8UC1);
    cv::Point2f iterPoint = currPoint;

    int iterCnt = 0;
    double err = 0;
    //迭代
    do
    {
        a = b = c = bb1 = bb2 = 0;
        //提取以当前点为中心的窗口子图像（为了方便求sobel微分，窗口各向四个方向扩展一行（列）像素）
        if ( !getSubImg(srcImg, iterPoint, subImg)) break;
        uchar *pSubData = (uchar*)subImg.data+winW+3;
        //如下计算参考上述推导公式，窗口内累加
        for (int i = 0; i < winH; i++)
        {
            for (int j = 0; j < winW; j++)
            {
                //读取高斯权重值
                double m = weightMask.at<float>(i, j);
                //sobel算子求梯度
                double sobelx = double(pSubData[i*(winW+2) + j + 1] - pSubData[i*(winW+2) + j - 1]);
                double sobely = double(pSubData[(i+1)*(winW+2) + j] - pSubData[(i - 1)*(winW+2) + j]);
                double gxx = sobelx*sobelx*m;
                double gxy = sobelx*sobely*m;
                double gyy = sobely*sobely*m;
                a += gxx;
                b += gxy;
                c += gyy;
                //邻域像素p的位置坐标
                double px = j - winSize.width;
                double py = i - winSize.height;

                bb1 += gxx*px + gxy*py;
                bb2 += gxy*px + gyy*py;
            }
        }
        double det = a*c - b*b;
        if (fabs(det) <= DBL_EPSILON*DBL_EPSILON)
            break;
        //求逆矩阵
        double invA = c / det;
        double invC = a / det;
        double invB = -b / det;
        //角点新位置
        cv::Point2f newPoint;
        newPoint.x = (float)(iterPoint.x + invA*bb1 + invB*bb2);
        newPoint.y = (float)(iterPoint.y + invB*bb1 + invC*bb2);
        //和上一次迭代之间的误差
        err = (newPoint.x - iterPoint.x)*(newPoint.x - iterPoint.x) + (newPoint.y - iterPoint.y)*(newPoint.y - iterPoint.y);
        //更新角点位置
        iterPoint = newPoint;
        iterCnt++;
        if (iterPoint.x < 0 || iterPoint.x >= srcImg.cols || iterPoint.y < 0 || iterPoint.y >= srcImg.rows)
            break;
    } while (err > eps && iterCnt < max_iters);
    //判断求得的亚像素角点与初始角点之间的差异，即：最小二乘法的收敛性
    if (fabs(iterPoint.x - currPoint.x) > winSize.width || fabs(iterPoint.y - currPoint.y) > winSize.height)
        iterPoint = currPoint;
    //保存算出的亚像素角点
    currPoint = iterPoint;
}


bool ArmorFinder::convertToStereoscopicCoordinate(
        cv::Rect2d &armor_box_left, cv::Rect2d &armor_box_right, cv::Point3d &space_position) {

    //cv::Point2d currPoint1, currPoint2;
    //TermCriteria criteria = TermCriteria(
     //       CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
     //       40, //maxCount=40
     //       0.001);    //epsilon=0.001

    //currPoint1.x = armor_box_left.x;
    //currPoint1.y = armor_box_left.y;
    //currPoint2.x = armor_box_right.x;
    //currPoint2.y = armor_box_right.y;

    //myCornerSubPix(src_raw_left_, currPoint1, Size(20,20),Size(-1,-1),criteria);
    //myCornerSubPix(src_raw_right_,currPoint2, Size(20,20),Size(-1,-1),criteria);

    double disparity, distance; //视差
    //disparity = abs(currPoint1.x - currPoint2.x) * stereo_camera_param_.LENGTH_PER_PIXAL;
    disparity = abs(armor_box_left.x - armor_box_right.x);// * stereo_camera_param_.LENGTH_PER_PIXAL;

    //space_position.z = (-1.167*log(abs(currPoint1.x - currPoint2.x))+5.2268)*100;  // this is subject to cameras changes
    //space_position.z = stereo_camera_param_.CAMERA_DISTANCE * stereo_camera_param_.FOCUS / disparity ;
    distance = 198 * 538 / disparity;  // baseline * fx / disparity
    distance = max(500.0, distance);
    distance = min(3000.0, distance);
    space_position.z = distance;
    space_position.x = space_position.z * (armor_box_left.x + armor_box_left.width/2 - 321) / 538;
                      
    space_position.y = space_position.z * (armor_box_right.y + armor_box_right.height/2 - 246) / 538;
                    

    //space_position.x -= stereo_camera_param_.CAMERA_DISTANCE/2;
    //std::cout << armor_space_position_ << std::endl;
    

    return true;
}

