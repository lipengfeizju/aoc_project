#ifndef FLA_DPM_DETECTOR_H_
#define FLA_DPM_DETECTOR_H_

#include "ros/ros.h"
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <math.h>
#include "dpm_detect_client.h"

#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_geometry/stereo_camera_model.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_datatypes.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "undistorter.h"

#include <XmlRpcValue.h>

namespace fla_dpm_detector {

class FlaDpmDetector {
public:
  FlaDpmDetector(ros::NodeHandle nh_in, int param_option,
                 double starting_distance_in, bool debug_in,
		 XmlRpc::XmlRpcValue camera_params, double error_tol_in);
  ~FlaDpmDetector();

  bool consistencyCheck();

  double rate;
  bool detector_started, debug;

protected:
  void setCameraParams(XmlRpc::XmlRpcValue camera_params);
  void startImageSub();
  void odomCb(const nav_msgs::Odometry::ConstPtr &odom_in);
  void imgCb(const sensor_msgs::ImageConstPtr& l_image_msg,
	     const sensor_msgs::ImageConstPtr& r_image_msg);

  ros::NodeHandle nh;
  ros::Subscriber cam_odom_sub, image_sub;
  image_transport::Publisher image_pub, pub_rect_left_, pub_rect_right_;
  ros::Publisher disp_pub, pose_pub;
  image_transport::ImageTransport it;

  dpm_detect_client::DpmDetectClient dpm_client;

  geometry_msgs::Pose goal_pose;

  double starting_distance, resize_val, error_tol;
  int last_valid;
  
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
  image_geometry::PinholeCameraModel left_model_, right_model_;;

  geometry_msgs::Pose cam_odom_pose;
  std::vector<bool> valid_pose;
  std::vector<tf::Transform> cam_odom_vec;
  std::vector<tf::Transform> polaris_pos_vec;

  boost::shared_ptr<ExactSync> exact_sync_;
  image_geometry::StereoCameraModel stereo_model_;
  
  cv::StereoBM block_matcher_;

  cv::Mat mapX_l_equi, mapY_l_equi, mapX_r_equi, mapY_r_equi;
  cv::Mat mapX_l_rect, mapY_l_rect, mapX_r_rect, mapY_r_rect;
  double f,b,cx,cy;
  bool match_found, odom_received;
  int num_ima;
};
}

#endif
