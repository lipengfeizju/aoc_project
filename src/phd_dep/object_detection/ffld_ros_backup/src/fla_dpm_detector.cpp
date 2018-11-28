#ifndef FLA_DPM_DETECTOR_CPP_
#define FLA_DPM_DETECTOR_CPP_

#include "ffld_ros/fla_dpm_detector.h"

namespace fla_dpm_detector {
FlaDpmDetector::FlaDpmDetector(ros::NodeHandle nh_in, int param_option,
                               double starting_distance_in, bool debug_in,
			       XmlRpc::XmlRpcValue camera_params, double error_tol_in)
  : dpm_client(nh_in), it(nh_in), block_matcher_(cv::StereoBM::BASIC_PRESET) {

  nh = nh_in;

  image_pub = it.advertise("detections", 1);
  /*pub_rect_left_ = it.advertise("left_rect", 1);
    pub_rect_right_ = it.advertise("right_rect", 1);*/
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("polaris_pos", 1);
  //disp_pub = nh.advertise<stereo_msgs::DisparityImage>("disparity", 1);
  
  dpm_detect_client::Params params;

  params.model_num = param_option;
  params.overlap = 0.5;
  params.threshold = -0.5;
  params.padding = 6;
  params.interval = 3;
  params.resize_val = 0.5;
  rate = 1;

  error_tol = error_tol_in;
  debug = debug_in;
  
  goal_pose.position.x = 0;
  goal_pose.position.y = 0;
  goal_pose.position.z = 20;

  num_ima = 0;
  match_found = false;
  odom_received = true;
  /*if (param_option == 1) {
    params.model_num = 0;
    params.overlap = 0.5;
    params.threshold = -0.5;
    params.padding = 6;
    params.interval = 3;
    params.resize_val = 0.5;
    rate = 1;
  } else {
    params.model_num = 0;
    params.overlap = 0.5;
    params.threshold = -0.5;
    params.padding = 6;
    params.interval = 3;
    params.resize_val = 0.5;
    rate = 1;
    }*/
  
  resize_val = params.resize_val;
  
  dpm_client.setDpmParams(params);

  starting_distance = starting_distance_in;
  
  block_matcher_.state->SADWindowSize = 31;
  block_matcher_.state->numberOfDisparities = 32;
  block_matcher_.state->preFilterSize = 5;
  block_matcher_.state->preFilterCap = 25;
  block_matcher_.state->minDisparity = 0;
  block_matcher_.state->textureThreshold = 100;
  block_matcher_.state->uniquenessRatio = 30;
  block_matcher_.state->speckleWindowSize = 100;
  block_matcher_.state->speckleRange = 1;
  block_matcher_.state->disp12MaxDiff = 0;

  setCameraParams(camera_params);

  /*if (!debug) {
    geometry_msgs::PoseStamped::ConstPtr goal_in =
      ros::topic::waitForMessage<geometry_msgs::PoseStamped>("goal_pose", nh);
    goal_pose = goal_in->pose;
    cam_odom_sub = nh.subscribe("odom", 1, &FlaDpmDetector::odomCb, this);
    } else {
    ROS_DEBUG("[fla_dpm_detector]: Starting detector in debug mode");*/
  cam_odom_sub = nh.subscribe("odom", 1, &FlaDpmDetector::odomCb, this);
  startImageSub();
    //}
}

  void FlaDpmDetector::setCameraParams(XmlRpc::XmlRpcValue camera_params) {
    std::vector<double> intrinsics_l, intrinsics_r, dist_l, dist_r;
    double t_b_c_l[16];
    double t_b_c_r[16];
    Eigen::Matrix4d T_B_C_l, T_B_C_r;
    for (int i=0;i<4;i++){
      intrinsics_l.push_back(static_cast<double>(camera_params[0]["camera"]["intrinsics"]["data"][i]));
      intrinsics_r.push_back(static_cast<double>(camera_params[1]["camera"]["intrinsics"]["data"][i]));
      dist_l.push_back(static_cast<double>(camera_params[0]["camera"]["distortion"]["data"][i]));
      dist_r.push_back(static_cast<double>(camera_params[1]["camera"]["distortion"]["data"][i]));
      for (int j=0;j<4; j++){
	T_B_C_l(i,j) = static_cast<double>(camera_params[0]["T_B_C"]["data"][i*4+j]);
	T_B_C_r(i,j) = static_cast<double>(camera_params[1]["T_B_C"]["data"][i*4+j]);
      }
    }

    Eigen::Matrix4d H = T_B_C_r.inverse()*T_B_C_l;

    Eigen::Matrix3d R = H.block<3,3>(0,0);
    Eigen::Vector3d t = H.block<3,1>(0,3);
    
    int width = static_cast<int>(camera_params[0]["camera"]["image_width"]);
    int height = static_cast<int>(camera_params[0]["camera"]["image_height"]);

    Eigen::Vector2d focalLength_l(intrinsics_l[0], intrinsics_l[1]);
    Eigen::Vector2d principalPoint_l(intrinsics_l[2], intrinsics_l[3]);
    Eigen::Vector2i resolution_l(width, height);
    Eigen::Vector4d distCoeffs_Equi_l(dist_l[0],dist_l[1],dist_l[2],dist_l[3]);

    undistorter::PinholeGeometry camera_l(focalLength_l, principalPoint_l, resolution_l, undistorter::EquidistantDistortion::create(distCoeffs_Equi_l));
    
    Eigen::Vector2d focalLength_r(intrinsics_r[0], intrinsics_r[1]);
    Eigen::Vector2d principalPoint_r(intrinsics_r[2],intrinsics_r[3]);
    Eigen::Vector2i resolution_r(width, height);
    Eigen::Vector4d distCoeffs_Equi_r(dist_r[0],dist_r[1],dist_r[2],dist_r[3]);

    undistorter::PinholeGeometry camera_r(focalLength_r, principalPoint_r, resolution_r, undistorter::EquidistantDistortion::create(distCoeffs_Equi_r));
    
    cv::Mat_<double> d_l_rect(5,1);
    cv::Mat_<double> d_r_rect(5,1);
    d_l_rect << 0,0,0,0,0;
    d_r_rect << 0,0,0,0,0;
    
    cv::Mat Rcv, tcv;
    cv::eigen2cv(R,Rcv);
    cv::eigen2cv(t,tcv);
    
    cv::Mat R_l, R_r, P_l, P_r, Q;

    Eigen::Matrix3d optimal_l = undistorter::cv_helper::getOptimalNewCameraMatrix(camera_l, cvSize(width,height), 0.0, cvSize(width, height));
    Eigen::Matrix3d optimal_r = undistorter::cv_helper::getOptimalNewCameraMatrix(camera_r, cvSize(width,height), 0.0, cvSize(width, height));

    undistorter::cv_helper::initUndistortRectifyMap(camera_l, Eigen::Matrix3d::Identity(), optimal_l, cv::Size(width, height), CV_32FC1, mapX_l_equi, mapY_l_equi);
    undistorter::cv_helper::initUndistortRectifyMap(camera_r, Eigen::Matrix3d::Identity(), optimal_r, cv::Size(width, height), CV_32FC1, mapX_r_equi, mapY_r_equi);

    cv::Mat K_l, K_r;
    
    cv::eigen2cv(optimal_l, K_l);
    cv::eigen2cv(optimal_r, K_r);

    cv::stereoRectify(K_l, d_l_rect, K_r, d_r_rect, cv::Size(1280,1024), Rcv, tcv, R_l, R_r, P_l, P_r, Q);

    f = P_l.at<double>(cv::Point(0,0));
    b = -P_r.at<double>(cv::Point(3,0))/f;
    cx = P_l.at<double>(cv::Point(2,0));
    cy = P_l.at<double>(cv::Point(2,1));
    
    cv::initUndistortRectifyMap(K_l, d_l_rect, R_l, P_l, cv::Size(width, height), CV_32FC1, mapX_l_rect, mapY_l_rect);
    cv::initUndistortRectifyMap(K_r, d_r_rect, R_r, P_r, cv::Size(width, height), CV_32FC1, mapX_r_rect, mapY_r_rect);
  }

  void FlaDpmDetector::odomCb(const nav_msgs::Odometry::ConstPtr &odom_in) {
    odom_received = true;
    cam_odom_pose = odom_in->pose.pose;
    /*if (sqrt(pow(cam_odom_pose.position.x - goal_pose.position.x, 2) +
	     pow(cam_odom_pose.position.y - goal_pose.position.y, 2) +
	     pow(cam_odom_pose.position.z - goal_pose.position.z, 2)) && !detector_started) {
      startImageSub();
      detector_started = true;
      }*/
  }

  void FlaDpmDetector::startImageSub(){
    exact_sync_.reset( new ExactSync(ExactPolicy(5),
                                     sub_l_image_,
                                     sub_r_image_));
    exact_sync_->registerCallback(boost::bind(&FlaDpmDetector::imgCb,
                                              this, _1, _2));
    sub_l_image_.subscribe(it, "left/image_raw", 1);
    sub_r_image_.subscribe(it, "right/image_raw", 1);
    
  }

  void FlaDpmDetector::imgCb(const sensor_msgs::ImageConstPtr& l_image_msg,
			     const sensor_msgs::ImageConstPtr& r_image_msg) {
    //ROS_INFO("[fla_dpm_detector]: Images received");
    
    /*if (!debug && !detector_started) {
      return;
      }*/

    cv::Mat image_left = cv_bridge::toCvCopy(l_image_msg)->image;
    cv::Mat image_right = cv_bridge::toCvCopy(r_image_msg)->image;
    cv::Mat rect_left, rect_right;

    // Rectify images based on previously computed rectification maps
    cv::remap(image_left, image_left, mapX_l_equi, mapY_l_equi, cv::INTER_LINEAR);
    cv::remap(image_right, image_right, mapX_r_equi, mapY_r_equi, cv::INTER_LINEAR);
        
    cv::remap(image_left, rect_left, mapX_l_rect, mapY_l_rect, cv::INTER_LINEAR);
    cv::remap(image_right, rect_right, mapX_r_rect, mapY_r_rect, cv::INTER_LINEAR);
    
    /*
    // Uncomment to write images to file
    std::ostringstream oss2;
    oss2 << "/home/alex/FLA_git/data/left_" << num_ima << ".png";
    std::string left_var = oss2.str();
    std::ostringstream oss3;
    oss3 << "/home/alex/FLA_git/data/right_" << num_ima << ".png";
    std::string right_var = oss3.str(); 
    
    cv::imwrite(left_var, rect_left);
    cv::imwrite(right_var, rect_right);

    num_ima++;
    */
    ros::Time detect_start = ros::Time::now();

    cv_bridge::CvImage rect_left_msg;
    rect_left_msg.header = l_image_msg->header;
    rect_left_msg.image = rect_left;
    rect_left_msg.encoding = l_image_msg->encoding;

    // Call detection on left image only
    ffld_ros::DetectObject detections = dpm_client.requestDetection(*(rect_left_msg.toImageMsg()));
    
    //ROS_DEBUG("[fla_dpm_detector]: dpm detection took %f seconds\n", (ros::Time::now()-detect_start).toSec());

    // Publish detection image
    if (image_pub.getNumSubscribers() > 0) {
      cv::Mat detection_image = dpm_client.drawDetection(rect_left, detections);
      image_pub.publish(cv_bridge::CvImage(l_image_msg->header, "bgr8",
					   detection_image).toImageMsg());
    }

    // If no detection, just return
    if (detections.response.left.size()==0) {
      return;
    }   

    double center_x = (detections.response.left[0]+detections.response.right[0])/(resize_val*2);
    double center_y = (detections.response.top[0]+detections.response.bottom[0])/(resize_val*2);

    center_x = (center_x-cx)/f;
    center_y = (center_y-cy)/f;
 
    sensor_msgs::RegionOfInterest roi;
    int x_offset = std::max(round(detections.response.left[0]/resize_val), 1.0);
    int y_offset = std::max(round(detections.response.top[0]/resize_val), 1.0);
    int height = round((detections.response.bottom[0] - detections.response.top[0])/resize_val);
    int width = round((detections.response.right[0] - detections.response.left[0])/resize_val);
    
    bool istop = false;
    bool isleft = false;

    if (x_offset - block_matcher_.state->SADWindowSize >0){
      x_offset -= block_matcher_.state->SADWindowSize;
      isleft = true;
    }
    width += block_matcher_.state->SADWindowSize;
    
    if (y_offset-block_matcher_.state->SADWindowSize > 0){
      y_offset -= block_matcher_.state->SADWindowSize;
      istop = true;
    } 
    height += block_matcher_.state->SADWindowSize;
    
    cv::Rect detectROI(x_offset, y_offset, width, height);
    
    // Crop image to detection ROI                                                                                                                         
    rect_left = rect_left(detectROI);
    rect_right = rect_right(detectROI);
    
    cv::Mat disp_image;
    
    ros::Time disp_start = ros::Time::now();

    block_matcher_(rect_left, rect_right, disp_image, CV_32F);
       
    double maxVal;
    cv::Point maxLoc;
    
    cv::minMaxLoc(disp_image, 0, &maxVal, 0, &maxLoc);

    if (maxVal <= 0){
      return;
    }

    double depth = f*b/maxVal;
    /*
    
    
    if (!consistencyCheck()){
      return;
    }
    */
    
    tf::Transform cam_odom;
    cam_odom.setOrigin(tf::Vector3(cam_odom_pose.position.x, cam_odom_pose.position.y, cam_odom_pose.position.z));
    cam_odom.setRotation(tf::Quaternion(cam_odom_pose.orientation.x, cam_odom_pose.orientation.y, cam_odom_pose.orientation.z, cam_odom_pose.orientation.w));
    cam_odom_vec.push_back(cam_odom);
    
    
    tf::Transform polaris_pos;
    polaris_pos.setOrigin(tf::Vector3(center_x*depth, center_y*depth, depth));
    polaris_pos.setRotation(tf::Quaternion(0,0,0,1));
    polaris_pos_vec.push_back(polaris_pos);

    bool consistency = consistencyCheck();

    ROS_INFO_NAMED("polaris_pose", "[fla_dpm_detector]: Detected polaris at: %f, %f, %f", center_x*depth, center_y*depth, depth);

    geometry_msgs::PoseStamped posemsg;
    geometry_msgs::Pose pose;
    pose.position.x = center_x*depth;
    pose.position.y = center_y*depth;
    pose.position.z = depth;
    pose.orientation.w = 1;
    posemsg.pose = pose;
    posemsg.header.stamp = ros::Time::now();
    posemsg.header.frame_id = l_image_msg->header.frame_id;

    pose_pub.publish(posemsg);
    
    //ROS_DEBUG("[fla_dpm_detector]: Time for disparity matching is: %f\n", (ros::Time::now()-disp_start).toSec());    
    /*
    // Uncomment to display images live
    cv::circle(rect_left, maxLoc, 5, cv::Scalar(255), 3);

    cv::normalize(disp_image, disp_image, 0, 255, CV_MINMAX, CV_8U);
    
    cv::imshow("Left", rect_left);
    cv::imshow("Right", rect_right);
    cv::imshow("Disparity", disp_image);
    cv::waitKey(10);
    */
  }

  bool FlaDpmDetector::consistencyCheck() {
    // PERFORMS CONSISTENCY CHECK, HIGHLY EXPERIMENTAL
    // First run, just return
    if (polaris_pos_vec.size()==1 || !odom_received){
      return false;
    }
    
    tf::Transform curr_pos = polaris_pos_vec.back();
    
   if (!match_found) {
      for (int i=0; i<polaris_pos_vec.size()-1; i++){
	tf::Vector3 transformed_pos = (cam_odom_vec[i].inverseTimes(curr_pos)).getOrigin();
	tf::Vector3 orig_pos = polaris_pos_vec[i].getOrigin();
	double transform_error = transformed_pos.distance(orig_pos);
	//printf("Transformed_pos is: %f, %f, %f\n", transformed_pos.getX(), transformed_pos.getY(), transformed_pos.getZ());
	ROS_INFO_NAMED("consistency_check", "[fla_dpm_detector]: Transform error before match is: %f", transform_error);
	if (transform_error < error_tol) {
	  match_found = true;
	  last_valid = polaris_pos_vec.size()-1;
	  return true;
	}
      }
    } else {
      tf::Vector3 transformed_pos = (cam_odom_vec[last_valid].inverseTimes(curr_pos)).getOrigin();
      tf::Vector3 orig_pos = polaris_pos_vec[last_valid].getOrigin();
      double transform_error = transformed_pos.distance(orig_pos);
      ROS_INFO_NAMED("consistency_check", "[fla_dpm_detector]: Transform error after match is: %f", transform_error);
      if (transform_error < error_tol) {
	last_valid = polaris_pos_vec.size()-1;
	return true;
      }
    }
    return false;
  }

  FlaDpmDetector::~FlaDpmDetector() {}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fla_dpm_detector");
  
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  double starting_distance, error_tol;
  int param_option;
  bool debug;
  std::string cinfo_left_url, cinfo_right_url;
  XmlRpc::XmlRpcValue camera_info;
  nh_priv.param<double>("starting_distance", starting_distance);
  nh_priv.param<int>("detection_mode", param_option, 1);
  nh_priv.param<bool>("debug", debug, true);
  nh_priv.param<double>("consistency_tol", error_tol, true);
  nh_priv.getParam("cameras", camera_info);

  fla_dpm_detector::FlaDpmDetector detector(nh, param_option,
                                            starting_distance, debug, camera_info, error_tol);
  
  /*  ros::Rate loop_rate(10);
      if (detector.debug) {
    ros::Rate new_rate(detector.rate);
    loop_rate = new_rate;
    }*/

  ros::Rate loop_rate(detector.rate);
  
  while (ros::ok()) {
   // Switch to polaris detection rate when detector started
    /*if (detector.detector_started && !detector.debug) {
      ros::Rate new_rate(detector.rate);
      loop_rate = new_rate;
      }*/
    ros::spinOnce();
    loop_rate.sleep();
  }
}
#endif
