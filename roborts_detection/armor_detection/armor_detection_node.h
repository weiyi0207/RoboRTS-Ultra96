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

#ifndef ROBORTS_DETECTION_ARMOR_DETECTION_NODE_H
#define ROBORTS_DETECTION_ARMOR_DETECTION_NODE_H

#include <boost/thread.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/message_filter.h>
#include <tf/tf.h>
#include <tf/time_cache.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "actionlib/server/simple_action_server.h"
#include "roborts_msgs/ArmorDetectionAction.h"
#include "roborts_msgs/FricWhl.h"
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/GimbalRate.h"
#include "roborts_msgs/ShootCmd.h"
#include <sensor_msgs/image_encodings.h>

#include "alg_factory/algorithm_factory.h"
#include "cv_toolbox.h"
#include "io/io.h"
#include "roborts_msgs/ProjectileSupply.h"
#include "roborts_msgs/SupplierStatus.h"
#include "roborts_msgs/RobotHeat.h"
#include "state/node_state.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "armor_detection_algorithms.h"
#include "armor_detection_base.h"
#include "gimbal_control.h"
#include "proto/armor_detection.pb.h"

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <time.h>
#include <tracker/kcftracker.hpp>
#include "armor_finder/armor_finder.h"
#include "armor_finder/constant.h"
#include "camera/camera_wrapper.h"
#include "camera/video_wrapper.h"
#include "camera/wrapper_head.h"
#include "tools/calibrate_tool.h"
#include "tracker/tracker.h"
#define UNDETECDTHRESHOLD 20

namespace roborts_detection {

using roborts_common::ErrorInfo;
using roborts_common::NodeState;

class ArmorDetectionNode {
 public:
  explicit ArmorDetectionNode();
  /**
   * @brief Initializing armor detection algorithm.
   * @return Return the error information.
   */
  ErrorInfo Init();
  /**
   * @brief Actionlib server call back function.
   * @param data Command for control the algorithm thread.
   */
  void ActionCB(const roborts_msgs::ArmorDetectionGoal::ConstPtr& data);
  /**
   * @brief Starting the armor detection thread.
   */
  void StartThread();
  /**
   * @brief Pausing the armor detection thread when received command 2 in
   * action_lib callback function.
   */
  void PauseThread();
  /**
   * @brief Stopping armor detection thread.
   */
  void StopThread();
  /**
   * @brief Executing the armor detection algorithm.
   */
  void ExecuteLoop();
  /**
   * @brief Publishing enemy pose information that been calculated by the armor
   * detection algorithm.
   */
  void PublishMsgs();
  void TransformBbox();
  void RosSpin();

  ~ArmorDetectionNode();

 protected:

 private:
  std::shared_ptr<ArmorDetectionBase> armor_detector_;
  std::thread armor_detection_thread_;
  std::thread transform_bbox_thread_;
  std::thread spin_thread_;
  unsigned int max_rotating_fps_;
  unsigned int min_rotating_detected_count_;
  unsigned int undetected_armor_delay_;

  //! state and error
  NodeState node_state_;
  ErrorInfo error_info_;
  bool initialized_;
  bool running_;
  std::mutex mutex_;
  std::condition_variable condition_var_;
  unsigned int undetected_count_;

  //! enemy information
  double x_;
  double y_;
  double z_;
  bool detected_enemy_;
  unsigned long demensions_;
  tf::TransformBroadcaster br_enemy;
  tf::Transform transform_enemy_to_map;

  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle enemy_nh_;
  ros::Publisher enemy_info_pub_;

  ros::ServiceClient enemy_nh_firc;
  ros::ServiceClient enemy_nh_fire;
  roborts_msgs::FricWhl fric_whl;
  roborts_msgs::ShootCmd shoot_cmd;

  std::shared_ptr<CVToolbox> cv_toolbox_;
  actionlib::SimpleActionServer<roborts_msgs::ArmorDetectionAction> as_;
  roborts_msgs::GimbalAngle gimbal_angle_;

  //! control model
  GimbalContrl gimbal_control_;
  // Scan Callback

  float distance_scan;
  float x_scan, y_scan;

  // enemy poseXY average
  std::deque<float> q_enemy_x;
  std::deque<float> q_enemy_y;

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void RobotHeatCallback(const roborts_msgs::RobotHeat& msg);
  int heat;
  //void FakeEnemyCB(const geometry_msgs::PoseStamped& msg);
  // patrol angles;
  int patrol_yaw_index;
  float patrol_yaw[180];

  tf::TransformListener listener;
  tf::StampedTransform transform;
  float t[4][1] = {{0.0}, {0.0}, {5000.0}, {1}};
  cv::Mat target_in_gimbal = cv::Mat(4, 1, CV_32FC1, t);
  double yaw_now, pitch_now, roll_now;
  double yaw_last1 = 0, yaw_last2 = 0;
  double pitch_last1 =0, pitch_last2 =0;
  tf::Transformer TF;

  int fps;
  time_t cur_time;
  // Public topic enemy
  ros::NodeHandle n_enemy;
  ros::Publisher enemy_pub_pose =
      n_enemy.advertise<geometry_msgs::PoseStamped>("enemy_base_link", 10);

  ros::Publisher enemy_pub =
      n_enemy.advertise<geometry_msgs::Pose2D>("find_enemy", 10);
  ros::Subscriber sub =
      n_enemy.subscribe("scan", 1, &ArmorDetectionNode::laserCallback, this);
  ros::Subscriber heat_sub = 
      n_enemy.subscribe("robot_heat",1,&ArmorDetectionNode::RobotHeatCallback, this);
  //ros::Subscriber fake_enemy_ = n_enemy.subscribe(
      //"/move_base_simple/goal", 1, &ArmorDetectionNode::FakeEnemyCB, this);
  ros::Publisher scan_pub =
      n_enemy.advertise<std_msgs::Float32MultiArray>("scan_tag", 2);
  geometry_msgs::Pose2D msg_enemy;
  geometry_msgs::PoseStamped msg_enemy_pose;
  std_msgs::Float32MultiArray scan_tag;
  //=============Yolo Rect================//

  //cv::Rect2d armor_box(0.0, 100, 100, 100);
  cv::Rect2d armor_box;
  bool receiveflag = false;
  int yolo_detected = -1;
  int undetected_count = 0;
  bool trackflag = false;
  void rectCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  ros::Subscriber yolo_sub = n_enemy.subscribe(
      "yolo_rect", 1000, &ArmorDetectionNode::rectCallback, this);
  cv::Point3d target_3d;
  bool patrol_start = true;
  bool yolo_first_detect = true;
};
     

}  // namespace roborts_detection

#endif  // ROBORTS_DETECTION_ARMOR_DETECTION_NODE_H
