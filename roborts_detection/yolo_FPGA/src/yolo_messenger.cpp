#include "yolo_messenger.h"
#include <unistd.h>
#include "yolo_detect.h"

#define default_color 0   //detect blue color
using namespace std;
using namespace cv;

Mat image_yolo;
bool receive_flag;

int imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  try {
    receive_flag = true;
    image_yolo = cv_bridge::toCvShare(msg, "bgr8")->image;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  return 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "yolo_FPGA_node",
            ros::init_options::NoSigintHandler);
    // camera publish
    ros::NodeHandle n_cam;
    image_transport::ImageTransport it(n_cam);
    image_transport::Publisher yolo_image_pub = it.advertise("camera_yolo", 1);
    image_transport::Subscriber sub_cam =
        it.subscribe("/left_camera", 1, imageCallback);
    ros::Publisher rect_pub = n_cam.advertise<std_msgs::Float32MultiArray>("yolo_rect", 2);
    
    Mat img;
    vector<float> yolo_data;

    int color_flag;
    //if (argv[1][0] == 'B')
        //color_flag = 0;
    //else if (argv[1][0] == 'R')
        //color_flag = 1;
    //else
    color_flag = default_color;
     
    int imshow_flag;
    if (argv[2] == "imshow")
        imshow_flag = 1;
    else
        imshow_flag = 0;
  
     
    while (ros::ok())
    {
       ros::spinOnce();
       if(receive_flag == true)
       {
           cv::cvtColor(image_yolo, img, cv::COLOR_BGR2RGB);
           yolo_data = yolo_detect(img, color_flag);
           cv::Mat tmp=img.clone();
           sensor_msgs::ImagePtr img_msg =
               cv_bridge::CvImage(std_msgs::Header(), "bgr8", tmp)
              .toImageMsg();  // bgr8 rgb8 mono8 mono16
           yolo_image_pub.publish(img_msg);
                   
           if (yolo_data.size() > 0) {
               std_msgs::Float32MultiArray msg;
               msg.data.push_back(yolo_data[0]);
               msg.data.push_back(yolo_data[1]);
               msg.data.push_back(yolo_data[2]);
               msg.data.push_back(yolo_data[3]);
               rect_pub.publish(msg);
           } else {
               std_msgs::Float32MultiArray msg;
               msg.data.push_back(-1);
               msg.data.push_back(-1);
               msg.data.push_back(-1);
               msg.data.push_back(-1);
               rect_pub.publish(msg);
           }

           if(imshow_flag) {
               cv::rectangle(img, Point(yolo_data[0], yolo_data[1]), Point(yolo_data[0]+yolo_data[2], yolo_data[1]+yolo_data[3]), Scalar(0, 255, 0),
               2, 1, 0);
           }
           receive_flag == false;

       }

     }
    ros::spin();
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();
    ros::waitForShutdown();
    return 0;
}
