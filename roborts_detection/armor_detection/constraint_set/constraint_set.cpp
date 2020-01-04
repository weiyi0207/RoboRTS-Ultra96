/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include "constraint_set.h"
#include "XF_Accel.h"

#include "timer/timer.h"
#include "io/io.h"

namespace roborts_detection {

ConstraintSet::ConstraintSet(std::shared_ptr<CVToolbox> cv_toolbox):
    ArmorDetectionBase(cv_toolbox){
  filter_x_count_ = 0;
  filter_y_count_ = 0;
  filter_z_count_ = 0;
  filter_distance_count_ = 0;
  filter_pitch_count_ = 0;
  filter_yaw_count_ = 0;
  read_index_ = -1;
  detection_time_ = 0;
  thread_running_ = false;

  LoadParam();
  error_info_ = ErrorInfo(roborts_common::OK);
}

void ConstraintSet::LoadParam() {
  //read parameters
  ConstraintSetConfig constraint_set_config_;
  std::string file_name = ros::package::getPath("roborts_detection") + \
      "/armor_detection/constraint_set/config/constraint_set.prototxt";
  bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &constraint_set_config_);
  ROS_ASSERT_MSG(read_state, "Cannot open %s", file_name.c_str());

  enable_debug_ = constraint_set_config_.enable_debug();
  enemy_color_ = constraint_set_config_.enemy_color();
  using_hsv_ = constraint_set_config_.using_hsv();
  using_svm_ = constraint_set_config_.using_svm();
  create_dataset_ = constraint_set_config_.create_dataset();

  //armor info
  float armor_width = constraint_set_config_.armor_size().width();
  float armor_height = constraint_set_config_.armor_size().height();
  SolveArmorCoordinate(armor_width, armor_height);

  //algorithm threshold parameters
  light_max_aspect_ratio_ = constraint_set_config_.threshold().light_max_aspect_ratio();
  light_min_aspect_ratio_ = constraint_set_config_.threshold().light_min_aspect_ratio();
  light_max_area_ = constraint_set_config_.threshold().light_max_area();
  light_min_area_ = constraint_set_config_.threshold().light_min_area();
  light_max_angle_ = constraint_set_config_.threshold().light_max_angle();
  light_max_area_diff_ = constraint_set_config_.threshold().light_max_area_diff();
  light_max_angle_diff_ = constraint_set_config_.threshold().light_max_angle_diff();
  armor_max_angle_ = constraint_set_config_.threshold().armor_max_angle();
  armor_max_area_ = constraint_set_config_.threshold().armor_max_area();
  armor_min_area_ = constraint_set_config_.threshold().armor_min_area();
  armor_max_aspect_ratio_ = constraint_set_config_.threshold().armor_max_aspect_ratio();
  armor_max_pixel_val_ = constraint_set_config_.threshold().armor_max_pixel_val();
  armor_max_stddev_ = constraint_set_config_.threshold().armor_max_stddev();
  armor_max_mean_   = constraint_set_config_.threshold().armor_max_mean();

  color_thread_ = constraint_set_config_.threshold().color_thread();
  blue_thread_ = constraint_set_config_.threshold().blue_thread();
  red_thread_ = constraint_set_config_.threshold().red_thread();

  int get_intrinsic_state = -1;
  int get_distortion_state = -1;

  while ((get_intrinsic_state < 0) || (get_distortion_state < 0)) {
    ROS_WARN("Wait for camera driver launch %d", get_intrinsic_state);
    usleep(50000);
    ros::spinOnce();
    get_intrinsic_state = cv_toolbox_->GetCameraMatrix(intrinsic_matrix_);
    get_distortion_state = cv_toolbox_->GetCameraDistortion(distortion_coeffs_);
  }
}



ErrorInfo ConstraintSet::DetectArmor(bool &detected, cv::Point3f &target_3d) {
  std::vector<cv::RotatedRect> lights;
  std::vector<ArmorInfo> armors;

  auto img_begin = std::chrono::high_resolution_clock::now();
  bool sleep_by_diff_flag = true;
  while (true) {
    // Ensure exit this thread while call Ctrl-C
    if (!thread_running_) {
      ErrorInfo error_info(ErrorCode::STOP_DETECTION);
      return error_info;
    }
    read_index_ = cv_toolbox_->NextImage(src_img_);
    if (read_index_ < 0) {
      // Reducing lock and unlock when accessing function 'NextImage'
      if (detection_time_ == 0) {
        usleep(20000);
        continue;
      } else {
        double capture_time = 0;
        cv_toolbox_->GetCaptureTime(capture_time);
        if (capture_time == 0) {
          // Make sure the driver is launched and the image callback is called
          usleep(20000);
          continue;
        } else if (capture_time > detection_time_ && sleep_by_diff_flag) {
//          ROS_WARN("time sleep %lf", (capture_time - detection_time_));
          usleep((unsigned int)(capture_time - detection_time_));
          sleep_by_diff_flag = false;
          continue;
        } else {
          //For real time request when image call back called, the function 'NextImage' should be called.
          usleep(500);
          continue;
        }
      }
    } else {
      break;
    }
  }
  /*ROS_WARN("time get image: %lf", std::chrono::duration<double, std::ratio<1, 1000>>
      (std::chrono::high_resolution_clock::now() - img_begin).count());*/

  auto detection_begin = std::chrono::high_resolution_clock::now();
  TIMER_START(Detection)

    cv::cvtColor(src_img_, gray_img_, CV_BGR2GRAY);
    if (enable_debug_) {
      show_lights_before_filter_ = src_img_.clone();
      show_lights_after_filter_ = src_img_.clone();
      show_armors_befor_filter_ = src_img_.clone();
      show_armors_after_filter_ = src_img_.clone();
      cv::waitKey(1);
    }

    DetectLights(src_img_, lights);
    FilterLights(lights);
    PossibleArmors(lights, armors);
    FilterArmors(armors);
    if(!armors.empty()) {
      detected = true;
      ArmorInfo final_armor = SlectFinalArmor(armors);
      cv_toolbox_->DrawRotatedRect(src_img_, armors[0].rect, cv::Scalar(0, 255, 0), 2);
      CalcControlInfo(final_armor, target_3d);
      target_3d.x = (final_armor.rect.center.x - src_img_.cols / 2 - cv_toolbox_->GetWidthOffSet()) * (CV_PI / src_img_.cols);
      target_3d.y = (final_armor.rect.center.y - src_img_.rows / 2 - cv_toolbox_->GetHeightOffSet()) * (CV_PI / src_img_.rows);
    } else
      detected = false;
    if(enable_debug_) {
      cv::imshow("relust_img_", src_img_);
    }

  lights.clear();
  armors.clear();
  cv_toolbox_->ReadComplete(read_index_);
  ROS_INFO("read complete");
  detection_time_ = std::chrono::duration<double, std::ratio<1, 1000000>>
      (std::chrono::high_resolution_clock::now() - detection_begin).count();

  // std::cout << "DETECTION TIME " << detection_time_ << std::endl;
  TIMER_END(Detection)
  return error_info_;
}

void ConstraintSet::DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights) {
  //std::cout << "********************************************DetectLights********************************************" << std::endl;
  //TIMER_START(DetectLights_1)

  //cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  //cv::dilate(src, src, element, cv::Point(-1, -1), 1);
  cv::Mat binary_brightness_img, binary_light_img;
    float thresh;
    if (enemy_color_ == BLUE)
      thresh = blue_thread_;
    else
      thresh = red_thread_;

TIMER_START(PREPEOCESS_ACCEL)
lights = pre_process(src_img_, thresh, color_thread_, 255, enemy_color_);
TIMER_END(PREPEOCESS_ACCEL)

  //TIMER_END(DetectLights_1)

  //TIMER_START(DetectLights_2)

  //auto contours_light = cv_toolbox_->FindContours(binary_light_img);
  //auto contours_brightness = cv_toolbox_->FindContours(binary_brightness_img);

  //TIMER_END(DetectLights_2)

  //TIMER_START(DetectLights_3)

  //lights.reserve(contours_light.size());
  // TODO: To be optimized
  //std::vector<int> is_processes(contours_light.size());
  //for (unsigned int i = 0; i < contours_light.size(); ++i) {
   // for (unsigned int j = 0; j < contours_brightness.size(); ++j) {

   //     if (cv::pointPolygonTest(contours_brightness[j], contours_light[i][0], true) >= 0.0) {
   //       cv::RotatedRect single_light = cv::minAreaRect(contours_brightness[j]);
   //       lights.push_back(single_light);
   //       if (enable_debug_)
   //         cv_toolbox_->DrawRotatedRect(show_lights_before_filter_, single_light, cv::Scalar(0, 255, 0), 2);
   //       break;
   //     }
   // }
 // }

  //if (enable_debug_)
  //  cv::imshow("show_lights_before_filter", show_lights_before_filter_);

  //auto c = cv::waitKey(1);
  //if (c == 'a') {
  //  cv::waitKey(0);
 // }
  //TIMER_END(DetectLights_3)
}


void ConstraintSet::FilterLights(std::vector<cv::RotatedRect> &lights) {
  //std::cout << "********************************************FilterLights********************************************" << std::endl;
  TIMER_START(FilterLights)

  std::vector<cv::RotatedRect> rects;
  rects.reserve(lights.size());

  for (auto &light : lights) {
    auto height = std::max(light.size.width, light.size.height);
    auto width = std::min(light.size.width, light.size.height);
    auto angle = (light.angle + (light.size.width > light.size.height ? 90 : 0));

    auto ratio = height / (float)width;

    //std::cout << "light angle: " << angle << std::endl;
    //std::cout << "light_aspect_ratio: " << light_aspect_ratio << std::endl;
    //std::cout << "light_area: " << light.size.area() << std::endl;
    if (ratio > light_min_aspect_ratio_ &&
        ratio < light_max_aspect_ratio_ &&
        height * width <= light_max_area_ &&
        height * width >= light_min_area_ &&
        angle > -light_max_angle_ &&
        angle < light_max_angle_) {
          light.size.height = height;
          light.size.width = width;
          light.angle = angle;
          rects.push_back(light);
      if (enable_debug_)
        cv_toolbox_->DrawRotatedRect(show_lights_after_filter_, light, cv::Scalar(0, 255, 0), 2, angle);
    }
  }
  if (enable_debug_)
    cv::imshow("lights_after_filter", show_lights_after_filter_);

  lights = rects;

  TIMER_END(FilterLights)
}

void ConstraintSet::PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armors) {
  //std::cout << "********************************************PossibleArmors********************************************" << std::endl;
  TIMER_START(PossibleArmors)
  for (unsigned int i = 0; i < lights.size(); i++) {
    for (unsigned int j = i + 1; j < lights.size(); j++) {
      cv::RotatedRect light1 = lights[i];
      cv::RotatedRect light2 = lights[j];
      auto edge1 = std::minmax(light1.size.width, light1.size.height);
      auto edge2 = std::minmax(light2.size.width, light2.size.height);
      auto lights_dis = std::sqrt((light1.center.x - light2.center.x) * (light1.center.x - light2.center.x) +
          (light1.center.y - light2.center.y) * (light1.center.y - light2.center.y));
      auto center_angle = std::atan(std::abs(light1.center.y - light2.center.y) / std::abs(light1.center.x - light2.center.x)) * 180 / CV_PI;
      center_angle = center_angle > 90 ? 180 - center_angle : center_angle;
      //std::cout << "center_angle: " << center_angle << std::endl;

      cv::RotatedRect rect;
      rect.angle = static_cast<float>(center_angle);
      rect.center.x = (light1.center.x + light2.center.x) / 2;
      rect.center.y = (light1.center.y + light2.center.y) / 2;
      float armor_width = std::abs(static_cast<float>(lights_dis) - std::max(edge1.first, edge2.first));
      float armor_height = std::max<float>(edge1.second, edge2.second);

      rect.size.width = std::max<float>(armor_width, armor_height);
      rect.size.height = std::min<float>(armor_width, armor_height);

      float light1_angle = light1.angle; //light1.size.width < light1.size.height ? -light1.angle : light1.angle + 90
      float light2_angle = light2.angle; //light2.size.width < light2.size.height ? -light2.angle : light2.angle + 90
      //std::cout << "light1_angle: " << light1_angle << std::endl;
      //std::cout << "light2_angle: " << light2_angle << std::endl;
/*
      if (enable_debug_) {
        std::cout << "*******************************" << std::endl;
        std::cout << "light_angle_diff_: " << std::abs(light1_angle - light2_angle) << std::endl;
        std::cout << "radio: " << std::max<float>(edge1.second, edge2.second)/std::min<float>(edge1.second, edge2.second) << std::endl;
        std::cout << "armor_angle_: " << std::abs(center_angle) << std::endl;
        std::cout << "armor_aspect_ratio_: " << rect.size.width / (float) (rect.size.height) << std::endl;
        std::cout << "armor_area_: " << std::abs(rect.size.area()) << std::endl;
        std::cout << "armor_pixel_val_: " << (float)(gray_img_.at<uchar>(static_cast<int>(rect.center.y), static_cast<int>(rect.center.x))) << std::endl;
        std::cout << "pixel_y" << static_cast<int>(rect.center.y) << std::endl;
        std::cout << "pixel_x" << static_cast<int>(rect.center.x) << std::endl;
      }
*/
      //
      
      auto angle_diff = std::abs(light1_angle - light2_angle);
      // Avoid incorrect calculation at 180 and 0.
      if (angle_diff > 175) {
        angle_diff = 180 -angle_diff;
      }
      if (angle_diff < light_max_angle_diff_ &&
          std::max<float>(edge1.second, edge2.second)/std::min<float>(edge1.second, edge2.second) < 2.0 &&
          rect.size.width / (rect.size.height) < armor_max_aspect_ratio_ &&
          std::abs(rect.size.area()) > armor_min_area_ &&
          gray_img_.at<uchar>(static_cast<int>(rect.center.y), static_cast<int>(rect.center.x))
              < armor_max_pixel_val_) { //std::abs(center_angle) < armor_max_angle_ &&

        if (light1.center.x < light2.center.x) {
          std::vector<cv::Point2f> armor_points;
          CalcArmorInfo(armor_points, light1, light2);
          armors.emplace_back(ArmorInfo(rect, armor_points));
          if (enable_debug_)
            cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, rect, cv::Scalar(0, 255, 0), 2);
          armor_points.clear();
        } else {
          std::vector<cv::Point2f> armor_points;
          CalcArmorInfo(armor_points, light2, light1);
          armors.emplace_back(ArmorInfo(rect, armor_points));
          if (enable_debug_)
            cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, rect, cv::Scalar(0, 255, 0), 2);
          armor_points.clear();
        }
      }
    }
  }
  TIMER_END(PossibleArmors)
  if (enable_debug_)
    cv::imshow("armors_before_filter", show_armors_befor_filter_);
  
}

void ConstraintSet::FilterArmors(std::vector<ArmorInfo> &armors) {
  if (!using_svm_) return;
  TIMER_START(FilterArmors)
  //std::cout << "********************************************FilterArmors********************************************" << std::endl;
  // cv::Mat mask = cv::Mat::zeros(gray_img_.size(), CV_8UC1);
  for (auto armor_iter = armors.begin(); armor_iter != armors.end();) {
    // cv::Point pts[4];
    // for (unsigned int i = 0; i < 4; i++) {
    //   pts[i].x = (int) armor_iter->vertex[i].x;
    //   pts[i].y = (int) armor_iter->vertex[i].y;
    // }
    // cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255), 8, 0);

    // cv::Mat mat_mean;
    // cv::Mat mat_stddev;
    // cv::meanStdDev(gray_img_, mat_mean, mat_stddev, mask);

    // auto stddev = mat_stddev.at<double>(0, 0);
    // auto mean = mat_mean.at<double>(0, 0);
    //std::cout << "stddev: " << stddev << std::endl;
    //std::cout << "mean: " << mean << std::endl;

    cv::RotatedRect rect = armor_iter->rect;
    rect.size.width *= 1.2;
    rect.size.height *= 2.2;
    cv::Rect roi = rect.boundingRect();

    if (0 <= roi.x && 0 < roi.width && roi.x + roi.width <= gray_img_.cols && 0 <= roi.y && 0 < roi.height && roi.y + roi.height <= gray_img_.rows) {
      if (create_dataset_) {
        cv::imwrite("dataset/" + std::to_string(cv::getTickCount()) + ".jpg", gray_img_(roi));
      }

      if (svm_predictor_.Predict(gray_img_(roi))) {
        if (enable_debug_) {
          cv_toolbox_->DrawRotatedRect(show_armors_after_filter_, armor_iter->rect, cv::Scalar(0, 255, 0), 2);
        }
      } else {
        armor_iter = armors.erase(armor_iter);
        continue;
      }
    }

    armor_iter++;

    // if (stddev > armor_max_stddev_ || mean > armor_max_mean_) {
    //   armor_iter = armors.erase(armor_iter);
    // } else {
    //   armor_iter++;
    // }
  }

  // nms
  // std::vector<bool> is_armor(armors.size(), true);
  // for (int i = 0; i < armors.size() && is_armor[i] == true; i++) {
  //   for (int j = i + 1; j < armors.size() && is_armor[j]; j++) {
  //     float dx = armors[i].rect.center.x - armors[j].rect.center.x;
  //     float dy = armors[i].rect.center.y - armors[j].rect.center.y;
  //     float dis = std::sqrt(dx * dx + dy * dy);
  //     if (dis < armors[i].rect.size.width + armors[j].rect.size.width) {
  //       if (armors[i].rect.angle > armors[j].rect.angle) {
  //         is_armor[i] = false;
  //         //std::cout << "i: " << i << std::endl;
  //       } else {
  //         is_armor[j] = false;
  //         //std::cout << "j: " << j << std::endl;
  //       }
  //     }
  //   }
  // }
  // //std::cout << armors.size() << std::endl;
  // for (unsigned int i = 0; i < armors.size(); i++) {
  //   if (!is_armor[i]) {
  //     armors.erase(armors.begin() + i);
  //     is_armor.erase(is_armor.begin() + i);
  //     //std::cout << "index: " << i << std::endl;
  //   } else if (enable_debug_) {
  //     cv_toolbox_->DrawRotatedRect(show_armors_after_filter_, armors[i].rect, cv::Scalar(0, 255, 0), 2);
  //   }
  // }
  if (enable_debug_)
    cv::imshow("armors_after_filter", show_armors_after_filter_);

  TIMER_END(FilterArmors)
}

ArmorInfo ConstraintSet::SlectFinalArmor(std::vector<ArmorInfo> &armors) {
  std::sort(armors.begin(),
            armors.end(),
            [](const ArmorInfo &p1, const ArmorInfo &p2) { return p1.rect.size.area() > p2.rect.size.area(); });

  return armors[0];
}

void ConstraintSet::CalcControlInfo(const ArmorInfo & armor, cv::Point3f &target_3d) {
  cv::Mat rvec;
  cv::Mat tvec;
  cv::solvePnP(armor_points_,
               armor.vertex,
               intrinsic_matrix_,
               distortion_coeffs_,
               rvec,
               tvec);
  target_3d = cv::Point3f(tvec);

}

void ConstraintSet::CalcArmorInfo(std::vector<cv::Point2f> &armor_points,
                                 cv::RotatedRect left_light,
                                 cv::RotatedRect right_light) {
  cv::Point2f left_points[4], right_points[4];
  left_light.points(left_points);
  right_light.points(right_points);

  cv::Point2f right_lu, right_ld, lift_ru, lift_rd;
  std::sort(left_points, left_points + 4, [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x < p2.x; });
  std::sort(right_points, right_points + 4, [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x < p2.x; });
  if (right_points[0].y < right_points[1].y) {
    right_lu = right_points[0];
    right_ld = right_points[1];
  } else {
    right_lu = right_points[1];
    right_ld = right_points[0];
  }

  if (left_points[2].y < left_points[3].y) {
    lift_ru = left_points[2];
    lift_rd = left_points[3];
  } else {
    lift_ru = left_points[3];
    lift_rd = left_points[2];
  }
  armor_points.push_back(lift_ru);
  armor_points.push_back(right_lu);
  armor_points.push_back(right_ld);
  armor_points.push_back(lift_rd);

}

void ConstraintSet::SolveArmorCoordinate(const float width,
                                         const float height) {
  armor_points_.emplace_back(cv::Point3f(-width/2, height/2,  0.0));
  armor_points_.emplace_back(cv::Point3f(width/2,  height/2,  0.0));
  armor_points_.emplace_back(cv::Point3f(width/2,  -height/2, 0.0));
  armor_points_.emplace_back(cv::Point3f(-width/2, -height/2, 0.0));
}

void ConstraintSet::SignalFilter(double &new_num, double &old_num, unsigned int &filter_count, double max_diff) {
  if(fabs(new_num - old_num) > max_diff && filter_count < 2) {
    filter_count++;
    new_num += max_diff;
  } else {
    filter_count = 0;
    old_num = new_num;
  }
}

void ConstraintSet::SetThreadState(bool thread_state) {
  thread_running_ = thread_state;
}

ConstraintSet::~ConstraintSet() {

}
} //namespace roborts_detection
