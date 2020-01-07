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

#include "armor_detection_node.h"
#include <unistd.h>
using std::cout;
using std::endl;
cv::Mat image_yolo;

int imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  try {
    image_yolo = cv_bridge::toCvShare(msg, "bgr8")->image;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  return 0;
}

namespace roborts_detection {

ArmorDetectionNode::ArmorDetectionNode()
    : node_state_(roborts_common::IDLE),
      demensions_(3),
      initialized_(false),
      detected_enemy_(false),
      undetected_count_(0),
      as_(nh_, "armor_detection_node_action",
          boost::bind(&ArmorDetectionNode::ActionCB, this, _1), false) {
  initialized_ = false;
  enemy_nh_ = ros::NodeHandle();
  if (Init().IsOK()) {
    initialized_ = true;
    node_state_ = roborts_common::IDLE;
  } else {
    ROS_ERROR("armor_detection_node initalized failed!");
    node_state_ = roborts_common::FAILURE;
  }
  as_.start();
  armor_box = cv::Rect2d(-1.0, 100.0, 100.0, 100.0);
}

ErrorInfo ArmorDetectionNode::Init() {
  enemy_info_pub_ =
      enemy_nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 2);
  ArmorDetectionAlgorithms armor_detection_param;
  enemy_nh_firc =
      enemy_nh_.serviceClient<roborts_msgs::FricWhl>("/cmd_fric_wheel");
  enemy_nh_fire = enemy_nh_.serviceClient<roborts_msgs::ShootCmd>("/cmd_shoot");
  shoot_cmd.request.mode = 0;
  shoot_cmd.request.number = 1;
  std::string file_name = ros::package::getPath("roborts_detection") +
                          "/armor_detection/config/armor_detection.prototxt";
  bool read_state =
      roborts_common::ReadProtoFromTextFile(file_name, &armor_detection_param);
  if (!read_state) {
    ROS_ERROR("Cannot open %s", file_name.c_str());
    return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  }
  gimbal_control_.Init(
      -81.9147,  //  armor_detection_param.camera_gimbal_transform().offset_x(),
      110,       // armor_detection_param.camera_gimbal_transform().offset_y(),
      195.6288,  // armor_detection_param.camera_gimbal_transform().offset_z(),
      0,    // armor_detection_param.camera_gimbal_transform().offset_pitch(),
      1.5,  // armor_detection_param.camera_gimbal_transform().offset_yaw(),
      15,   // armor_detection_param.projectile_model_info().init_v(),
      armor_detection_param.projectile_model_info().init_k());

  // create the selected algorithms
  std::string selected_algorithm = armor_detection_param.selected_algorithm();
  // create image receiver
  // cv_toolbox_
  // =std::make_shared<CVToolbox>(armor_detection_param.camera_name());
  // create armor detection algorithm
  // armor_detector_ =
  // roborts_common::AlgorithmFactory<ArmorDetectionBase,std::shared_ptr<CVToolbox>>::CreateAlgorithm
  //   (selected_algorithm, cv_toolbox_);

  undetected_armor_delay_ = armor_detection_param.undetected_armor_delay();
  // if (armor_detector_ == nullptr) {
  //  ROS_ERROR("Create armor_detector_ pointer failed!");
  //  return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  //} else
  for (int i = 0; i < 45; i++) {
    patrol_yaw[i] = i / 180.0 * 3.14159;
  }
  for (int i = 0; i < 45; i++) {
    patrol_yaw[45 + i] = (45 - i) / 180.0 * 3.14159;
  }
  for (int i = 0; i < 45; i++) {
    patrol_yaw[90 + i] = -i / 180.0 * 3.14159;
  }
  for (int i = 0; i < 45; i++) {
    patrol_yaw[135 + i] = -(45 - i) / 180.0 * 3.14159;
  }
  patrol_yaw_index = 0;

  return ErrorInfo(ErrorCode::OK);
}
void ArmorDetectionNode::RobotHeatCallback(const roborts_msgs::RobotHeat& msg){
  heat = msg.shooter_heat;
}


void ArmorDetectionNode::ActionCB(
    const roborts_msgs::ArmorDetectionGoal::ConstPtr &data) {
  roborts_msgs::ArmorDetectionFeedback feedback;
  roborts_msgs::ArmorDetectionResult result;
  bool undetected_msg_published = false;

  if (!initialized_) {
    feedback.error_code = error_info_.error_code();
    feedback.error_msg = error_info_.error_msg();
    as_.publishFeedback(feedback);
    as_.setAborted(result, feedback.error_msg);
    ROS_INFO("Initialization Failed, Failed to execute action!");
    return;
  }

  switch (data->command) {
    case 1:
      StartThread();
      break;
    case 2:
      PauseThread();
      break;
    case 3:
      StopThread();
      break;
    default:
      break;
  }
  ros::Rate rate(25);
  while (ros::ok()) {
    if (as_.isPreemptRequested()) {
      as_.setPreempted();
      return;
    }

    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (undetected_count_ != 0) {
        feedback.detected = true;
        feedback.error_code = error_info_.error_code();
        feedback.error_msg = error_info_.error_msg();
 
        feedback.enemy_pos.header.frame_id = "map";
        feedback.enemy_pos.header.stamp = ros::Time::now();

        feedback.enemy_pos.pose.position.x = x_;
        feedback.enemy_pos.pose.position.y = y_;
        feedback.enemy_pos.pose.position.z = z_;
        feedback.enemy_pos.pose.orientation.w = 1;
        as_.publishFeedback(feedback);
        undetected_msg_published = false;
      } else if (!undetected_msg_published) {
        feedback.detected = false;
        feedback.error_code = error_info_.error_code();
        feedback.error_msg = error_info_.error_msg();

        feedback.enemy_pos.header.frame_id = "camera0";
        feedback.enemy_pos.header.stamp = ros::Time::now();

        feedback.enemy_pos.pose.position.x = 0;
        feedback.enemy_pos.pose.position.y = 0;
        feedback.enemy_pos.pose.position.z = 0;
        feedback.enemy_pos.pose.orientation.w = 1;
        as_.publishFeedback(feedback);
        undetected_msg_published = true;
      }
    }
    rate.sleep();
  }
}

void ArmorDetectionNode::ExecuteLoop() {
  // undetected_count_ = undetected_armor_delay_;

  int enemy_color = ENEMY_BLUE;

  WrapperHead *video;
  video = new CameraWrapper;
  ROS_INFO("start to initialize.");
  if (video->init()) {
    ROS_INFO("Camera initialized successfully.");

  } else {
    ROS_INFO("Camera initialization failed.");
    return;
  }
  cv::Mat src_left, src_right;
  cv::Mat src_left_parallel, src_right_parallel;
  ArmorFinder armor_finder;
  armor_finder.setEnemyColor(enemy_color);
  for (int i = 0; i < 5; i++) {
    video->read(src_left, src_right);  // to eliminate the initial noise images
    video->read(src_left_parallel, src_right_parallel);
  }
  cout << "start working" << endl;
  time_t start = time(NULL);
  int cnt = 0;
  bool ok = true;
  // camera publish
  ros::NodeHandle n_cam;
  image_transport::ImageTransport it(n_cam);
  image_transport::Publisher camera_pub = it.advertise("left_camera", 1);
  image_transport::Subscriber sub_cam =
      it.subscribe("/camera_yolo", 1, imageCallback);

  ok = video->read(src_left, src_right);

  armor_finder.calibrate(src_left, src_right);

  KCFTracker kcf_tracker(false, true, false, false);
  // armor_finder.trackInit(kcf_tracker, src_left, armor_box);

  while (running_ && ok) {
    //#pragma omp parallel sections
    //        {
    //#pragma omp section
    {
      ok = video->read(src_left, src_right);
      armor_finder.calibrate(src_left, src_right);
      transform.getBasis().getEulerYPR(yaw_now, pitch_now, roll_now);
      cv::Mat tmp = src_left.clone();
      // cv::cvtColor(tmp,tmp,cv::COLOR_GRAY2BGR);
      // cv::imshow("test", tmp);
      // cv::waitKey(1);

      // tmp = cv::imread("/home/dji/Pictures/Screenshot from 2019-05-01
      // 15-10-34.png");
      sensor_msgs::ImagePtr img_msg =
          cv_bridge::CvImage(std_msgs::Header(), "bgr8", tmp)
              .toImageMsg();  // bgr8 rgb8 mono8 mono16
      camera_pub.publish(img_msg);
      ros::spinOnce();
    }
    //#pragma omp section
    //            {
    //                //armor_finder.run(src_left_parallel, src_right_parallel);
    //            }
    //        }
    //#pragma omp barrier

    //#pragma omp parallel sections
    //        {
    //#pragma omp section
    //            {ok = video->read(src_left_parallel, src_right_parallel);
    //             armor_finder.calibrate(src_left_parallel,
    //             src_right_parallel);}
    //#pragma omp section
    //            {
    // armor_finder.run(src_left, src_right);
    //            }
    //        }
    //#pragma omp barrier

    usleep(1);
    time_t t = time(NULL);
    if (cur_time != t) {
      cur_time = t;
      std::cout << "FPS: " << fps << std::endl;
      fps = 0;
    }
    fps++;

    if (node_state_ == NodeState::RUNNING) {
      //================================Yolo rect=================/
      ros::spinOnce();
      cout << "Receive: " << receiveflag << "yolo_detected: " << yolo_detected << "undetected ct: " << undetected_count
           << "Track flag: " << trackflag << "Detect enemy: " << detected_enemy_ <<  "box: " << armor_box << "patrol start: " << patrol_start << "patrol yaw index: " << patrol_yaw_index << endl;
      if (receiveflag == true && yolo_detected == -1) {
        undetected_count++;
        yolo_detected = 0;
      }
      if (undetected_count > UNDETECDTHRESHOLD) {
        undetected_count = 0;
        trackflag = false;
      }
      yolo_first_detect = false;
      if (receiveflag == true && yolo_detected == 1 &&
       armor_box.width >= 0 &&armor_box.height >= 0 &&
       armor_box.x >= 0 &&armor_box.y >= 0 && 
       armor_box.height+armor_box.y<src_left.rows && armor_box.width+armor_box.x<src_left.cols) {
        trackflag = true;
        yolo_detected = 0;
        undetected_count = 0;
        //cv::Mat gray;
        //cv::cvtColor(image_yolo, gray, cv::COLOR_BGR2GRAY);
        //armor_finder.trackInit(kcf_tracker, gray, armor_box);
        //armor_finder.trackInit(kcf_tracker, image_yolo, armor_box);
        yolo_first_detect = true;
        receiveflag = false;
      }
      if (trackflag &&
       armor_box.x >= 0 &&armor_box.y >= 0 &&
       armor_box.width >= 0 &&armor_box.height >= 0 && 
       armor_box.height+armor_box.y<src_left.rows && armor_box.width+armor_box.x<src_left.cols) {
        //cv::Mat gray;
        //cv::cvtColor(src_left, gray, cv::COLOR_BGR2GRAY);
        //armor_finder.track(kcf_tracker, gray, armor_box);
        //armor_finder.track(kcf_tracker, src_left, armor_box);
        //rectangle(src_left, armor_box, cv::Scalar(0, 255, 0), 10, cv::LINE_8);
        detected_enemy_ = true;
      } else {
        detected_enemy_ = false;
      }
      // cout<<armor_box<<endl;

      {
        //std::lock_guard<std::mutex> guard(mutex_);
        target_3d.y = 1;
        target_3d.z = 140 * 538 / armor_box.height;
        target_3d.x =
            target_3d.z * (armor_box.x + armor_box.width / 2 - 321) / 538;
      }

      //imshow("box", src_left);
      //cv::waitKey(1);
        
      TransformBbox();
    } else if (node_state_ == NodeState::PAUSE) {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_var_.wait(lock);
    }
  }
  delete video;
}

void ArmorDetectionNode::TransformBbox() {
  
  //while (true) {
    //if (node_state_ == NodeState::RUNNING) {
      // TODO

      // armor_finder.convertToStereoscopicCoordinate(
      // armor_finder.armor_box_left_, armor_finder.armor_box_right_,
      // target_3d);
      // {
      //   std::lock_guard<std::mutex> guard(mutex_);
      //   x_ = target_3d.x;
      //   y_ = target_3d.y;
      //   z_ = target_3d.z;
      // }
      // cv::imshow("src_left_",armor_finder.src_left_)	;
      try {
        listener.lookupTransform("gimbal", "base_link", ros::Time(0),
                                 transform);
      } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
      }
      //transform.getBasis().getEulerYPR(yaw_now, pitch_now, roll_now);

      // cout<<yaw_now<<" "<<pitch_now<<" "<<roll_now<<endl;
      cout << "Detect enemy: " << detected_enemy_ << endl;
      if (detected_enemy_) {
        //************************************Camera to
        // Map**********************************************//
        target_in_gimbal.at<float>(0, 0) =
            (target_3d.z + gimbal_control_.offset_.z);
        target_in_gimbal.at<float>(1, 0) =
            -(target_3d.x + gimbal_control_.offset_.x);
        target_in_gimbal.at<float>(2, 0) = 100;
        float tmp[4][4] = {
            {float(cos(-yaw_now)), float(-sin(-yaw_now)), 0.0, 0.0},
            {float(sin(-yaw_now)), float(cos(-yaw_now)), 0.0, 0.0},
            {0.0, 0.0, 1.0, 0},
            {0.0, 0.0, 0.0, 1.0000}};
        cv::Mat transform_gimbal2base = cv::Mat(4, 4, CV_32FC1, tmp);

        cv::Mat target_in_base = transform_gimbal2base * target_in_gimbal;

        cout << "Target In Came " << target_3d.x << " " << target_3d.y << " "
             << target_3d.z << endl;
        cout << "Target In Base " << target_in_base.at<float>(0, 0) << " "
             << target_in_base.at<float>(1, 0) << endl;
        x_scan = target_in_base.at<float>(0, 0);
        y_scan = target_in_base.at<float>(1, 0);

        ros::spinOnce();
        //distance_scan -= 0.15;
        //distance_scan = distance_scan > 0 ? distance_scan : 0.15;
        //cout << "distance_scan " << distance_scan << endl;
        //target_3d.z = distance_scan * 1000.0;
        distance_scan = target_3d.z / 1000.0;
        distance_scan -= 0.30;
        distance_scan = distance_scan > 0 ? distance_scan : 0.15;
        target_in_base.at<float>(0, 0) = distance_scan * 1000.0;

        try {
          listener.lookupTransform("map", "base_link", ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
          ROS_ERROR("%s", ex.what());
        }
        //transform.getBasis().getEulerYPR(yaw_now,pitch_now,roll_now);
        // cout<<"get xy " <<transform.getOrigin().x()<<"
        // "<<transform.getOrigin().y()<<endl; cout<<"get "<<yaw_now<<"
        // "<<pitch_now<<" "<<roll_now<<endl;

        double yaw_map, pitch_map, roll_map;
        transform.getBasis().getEulerYPR(yaw_map, pitch_map, roll_map);
        float tmp_base2map[4][4] = {
            {float(cos(-yaw_map)), float(-sin(-yaw_map)), 0.0, 0.0},
            {float(sin(-yaw_map)), float(cos(-yaw_map)), 0.0, 0.0},
            {0.0, 0.0, 1.0, 0},
            {0.0, 0.0, 0.0, 1.0000}};

        cout << "postion in map " << transform.getOrigin().x()
             << transform.getOrigin().y() << endl;
        cv::Mat base2map = cv::Mat(4, 4, CV_32FC1, tmp_base2map);
        cv::Mat target_in_map = base2map * target_in_base;
        target_in_map.at<float>(0, 0) =
            transform.getOrigin().x() + target_in_map.at<float>(0, 0) / 1000;

        target_in_map.at<float>(1, 0) =
            transform.getOrigin().y() + target_in_map.at<float>(1, 0) / 1000;
        cout << "Target In Maps " << target_in_map.at<float>(0, 0) << " "
             << target_in_map.at<float>(1, 0) << endl;

        //*************************************End***********************************//

        /********Public enemy pose**************/

        if (q_enemy_x.size() >= 30) {
          q_enemy_x.pop_front();
          q_enemy_y.pop_front();
        }
        q_enemy_x.push_back(target_in_map.at<float>(0, 0));
        q_enemy_y.push_back(target_in_map.at<float>(1, 0));
        msg_enemy.x = 0;
        msg_enemy.y = 0;
        for (int i = 0; i < q_enemy_x.size(); i++) {
          msg_enemy.x += q_enemy_x[i];
          msg_enemy.y += q_enemy_y[i];
        }

        msg_enemy.x /= q_enemy_x.size();
        msg_enemy.y /= q_enemy_x.size();

        msg_enemy.theta = 1;
        msg_enemy_pose.pose.position.x = msg_enemy.x;
        msg_enemy_pose.pose.position.y = msg_enemy.y;
        msg_enemy_pose.pose.position.z = 0.1;
        msg_enemy_pose.header.frame_id = "map";
        {
          std::lock_guard<std::mutex> guard(mutex_);
          x_ = msg_enemy.x;
          y_ = msg_enemy.y;
          undetected_count_ = 1;
        }
        enemy_pub_pose.publish(msg_enemy_pose);
        enemy_pub.publish(msg_enemy);
        //ros::spinOnce();
        yaw_now *= 180 / 3.1415;
        float pitch, yaw;
        gimbal_control_.Transform(target_3d, pitch, yaw);
        gimbal_angle_.yaw_mode = true;
        gimbal_angle_.pitch_mode = false;
        if (yaw_now > 45 || yaw_now < -45) {
          //    yaw = 0;
        }  // PID
        if (yolo_first_detect) {
          gimbal_angle_.yaw_angle = yaw * 0.01;
        } else {
          //gimbal_angle_.yaw_angle = yaw * 0.35 + yaw_last1 * 0.15;
          gimbal_angle_.yaw_angle = yaw * 0.1 + yaw_last1 * 0.01;
        }
        yaw_last1 = yaw;
        gimbal_angle_.pitch_angle =  0.1*pitch+0.0*pitch_last1+0.0*pitch_last2;
        //pitch_last2 = pitch_last1; 
        //pitch_last1 = pitch;

        //std::lock_guard<std::mutex> guard(mutex_);
        // undetected_count_ = undetected_armor_delay_;
        if(abs(yaw) < 0.1 && heat < 100){
          shoot_cmd.request.mode = 1;
          shoot_cmd.request.number = 1;
          enemy_nh_fire.call(shoot_cmd);
        }
        else{
          shoot_cmd.request.mode = 0;
          shoot_cmd.request.number = 0;
          enemy_nh_fire.call(shoot_cmd);
        }
        PublishMsgs();
        patrol_start = true;
      } else {
        undetected_count_ = 0;
        shoot_cmd.request.mode = 0;
        shoot_cmd.request.number = 0;
        enemy_nh_fire.call(shoot_cmd);
        /* gimbal_angle_.yaw_mode = true;
         gimbal_angle_.pitch_mode = true;

         gimbal_angle_.yaw_angle = 0;
         gimbal_angle_.pitch_angle = 0;

         undetected_count_--;
         PublishMsgs();
         */
        if (patrol_start) {
          if (yaw_now >= 0) {
            yaw_now = std::min(yaw_now, 45.0);
            patrol_yaw_index = int(yaw_now);
          } else {
            yaw_now = std::max(yaw_now, -45.0);
            patrol_yaw_index = int(yaw_now + 179);
          }
        }
        msg_enemy.theta = -1;
        enemy_pub.publish(msg_enemy);

        //ros::spinOnce();  // patrol mode
        //usleep(10000);
        yaw_last1 = 0;
        pitch_last1=0;
        pitch_last2=0;
        gimbal_angle_.yaw_mode = false;
        gimbal_angle_.pitch_mode = true;
        gimbal_angle_.yaw_angle = patrol_yaw[patrol_yaw_index];
        cout << "Patrol yaw index" << endl;
        patrol_yaw_index++;
        patrol_yaw_index %= 180;
        //gimbal_angle_.pitch_angle = 0;
        shoot_cmd.request.mode = 0;
        enemy_nh_fire.call(shoot_cmd);
        PublishMsgs();
        patrol_start = false;
      }
    //} else if (node_state_ == NodeState::PAUSE) {
      //std::unique_lock<std::mutex> lock(mutex_);
      //condition_var_.wait(lock);
    //}
  //}
}

void ArmorDetectionNode::RosSpin() {
  ros::Rate rate(1000);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}

void ArmorDetectionNode::PublishMsgs() {
  enemy_info_pub_.publish(gimbal_angle_);
}
void ArmorDetectionNode::StartThread() {
  ROS_INFO("Armor detection node started!");
  running_ = true;
  // armor_detector_->SetThreadState(true);
  if (node_state_ == NodeState::IDLE) {
    armor_detection_thread_ =
        std::thread(&ArmorDetectionNode::ExecuteLoop, this);
    //usleep(100);
    //transform_bbox_thread_ =
        //std::thread(&ArmorDetectionNode::TransformBbox, this);
    //usleep(100);
    spin_thread_ = std::thread(&ArmorDetectionNode::RosSpin, this);
  }
  node_state_ = NodeState::RUNNING;
  condition_var_.notify_one();
  fric_whl.request.open = true;
  if (enemy_nh_firc.call(fric_whl)) {
    ROS_INFO("Open firc wheel Succeed!");
  } else {
    ROS_ERROR("Fail to open fric wheel");
  }
}

void ArmorDetectionNode::PauseThread() {
  ROS_INFO("Armor detection thread paused!");
  node_state_ = NodeState::PAUSE;

  fric_whl.request.open = false;
  if (enemy_nh_firc.call(fric_whl)) {
    ROS_INFO("Close firc wheel Succeed!");
  } else {
    ROS_ERROR("Fail to close fric wheel");
  }
}

void ArmorDetectionNode::StopThread() {
  fric_whl.request.open = false;
  if (enemy_nh_firc.call(fric_whl)) {
    ROS_INFO("Close firc wheel Succeed!");
  } else {
    ROS_ERROR("Fail to close fric wheel");
  }
  node_state_ = NodeState::IDLE;
  running_ = false;
  // armor_detector_->SetThreadState(false);
  if (armor_detection_thread_.joinable()) {
    armor_detection_thread_.join();
  }
  //if (transform_bbox_thread_.joinable()) {
    //transform_bbox_thread_.join();
  //}
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  ROS_INFO("Armor detection thread stopped!");
}

void ArmorDetectionNode::laserCallback(
    const sensor_msgs::LaserScan::ConstPtr &msg) {
  std::vector<float> ranges = msg->ranges;
  float angle_min = msg->angle_min;
  float angle_max = msg->angle_max;
  float increment = msg->angle_increment;
  float scan_sum = 0;
  float angle_todo = atan2(y_scan, x_scan);
  float angel_now;
  int count = 0;
  int sizes = ranges.size();
  int idx = (angle_todo - angle_min) / increment;
  for (int i = std::max(0, idx - 3); i < std::min(idx + 3, sizes); ++i) {
    scan_sum = scan_sum + ranges[i];
    count++;
  }
  distance_scan = scan_sum / count;
//   scan_tag.data.clear();
//   int scale = ranges.size() / 135;
//   std::vector<float>::iterator iter = ranges.begin();
//   for (int i = 0; i < 135; i++) {
//     scan_tag.data.push_back(std::accumulate(iter, iter + scale, 0.0f) / 5.0f /
//                             (float)scale);
//     iter += scale;
//   }
//   int robot_ind = idx / scale;

//   for (int i = 0; i < 135; i++) {
//     scan_tag.data.push_back(0);
//   }
//   if (detected_enemy_) {
//     for (int i = 135 + std::max(0, robot_ind - 3);
//          i < 135 + std::min(135, robot_ind + 3); ++i) {
//       scan_tag.data.at(i) = 1;
//     }
//   }
//   scan_pub.publish(scan_tag);
}

void ArmorDetectionNode::rectCallback(
    const std_msgs::Float32MultiArray::ConstPtr &msg) {
  receiveflag = true;
  yolo_detected = msg->data[0] >= 0 ? 1 : -1;
  if (yolo_detected == 1) {
      armor_box = cv::Rect2d(double(msg->data[0]), double(msg->data[1]),
                         double(msg->data[2]), double(msg->data[3]));
  }
}

ArmorDetectionNode::~ArmorDetectionNode() {
  fric_whl.request.open = false;
  if (enemy_nh_firc.call(fric_whl)) {
    ROS_INFO("Close firc wheel Succeed!");
  } else {
    ROS_ERROR("Fail to close fric wheel");
  }
  StopThread();
}
}  // namespace roborts_detection

void SignalHandler(int signal) {
  if (ros::isInitialized() && ros::isStarted() && ros::ok() &&
      !ros::isShuttingDown()) {
    ros::shutdown();
  }
}
int main(int argc, char **argv) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);
  ros::init(argc, argv, "armor_detection_node",
            ros::init_options::NoSigintHandler);
  roborts_detection::ArmorDetectionNode armor_detection;
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  ros::waitForShutdown();
  armor_detection.StopThread();
  return 0;
}
