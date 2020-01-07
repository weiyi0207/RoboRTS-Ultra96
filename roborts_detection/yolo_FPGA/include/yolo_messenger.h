#ifndef YOLO_MESSENGER_H
#define YOLO_MESSENGER_H

#include <boost/thread.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
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
#include "io/io.h"
#include "roborts_msgs/ProjectileSupply.h"
#include "roborts_msgs/SupplierStatus.h"
#include "state/node_state.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <time.h>

#endif
