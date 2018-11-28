#include "ros/ros.h"
#include "ffld_ros/DetectObject.h"
#include "FFLDDetector.hpp"
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <fstream>

struct Params {
  std::string model_name;
  FFLD::Mixture mixture;
  FFLDDetector DPM;
  double overlap;    // Minimum overlap in non maxima suppression (default 0.5)  
  double threshold;  // Minimum detection threshold (default -1)
  int interval;      // Number of levels per octave in the HOG pyramid (default 5)
  int padding;       // Amount of zero padding in HOG cells (default 6)
} pr_;


bool detect_object(ffld_ros::DetectObject::Request  &req,
                   ffld_ros::DetectObject::Response &res)
{
  // Load image
  const FFLD::JPEGImage image( req.img.width, req.img.height, 
                               sensor_msgs::image_encodings::numChannels( req.img.encoding ),
                               req.img.data.data() ); 
  // Detect
  std::vector<Detection> detections;
  if( pr_.DPM.detectImg( pr_.mixture, image, detections, 
                             pr_.overlap, pr_.threshold, 
                             pr_.interval, pr_.padding ) == - 1 )
    return false;
  
  for (int i = 0; i < detections.size(); ++i)
  {
    res.score.push_back( detections[i].score );
    res.left.push_back( detections[i].left() + 1 ); // make index 1-based instead of 0-based
    res.top.push_back( detections[i].top() + 1 );
    res.right.push_back( detections[i].right() + 1 );
    res.bottom.push_back( detections[i].bottom() + 1 );
  }
  return true;  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_object_server");
  
  ros::NodeHandle pnh("~");
  pnh.param("overlap",    pr_.overlap,    0.5);
  pnh.param("threshold",  pr_.threshold, -1.0);
  pnh.param("interval",   pr_.interval,   5);
  pnh.param("padding",    pr_.padding,    6);
  pnh.param<std::string>("model_name", pr_.model_name, "object");
  
  // Try to read the model
  std::ifstream in(pr_.model_name.c_str(), std::ios::binary);
  if (!in.is_open()) {
    ROS_ERROR("[detect_object_server]: Invalid model file: %s", pr_.model_name.c_str());
	  return 1;
  }
  in >> pr_.mixture;
  if (pr_.mixture.empty()) {
    ROS_ERROR("[detect_object_server]: Invalid model file: %s", pr_.model_name.c_str());
	  return 1;
  }
  
  ROS_INFO("[detect_object_server]: Server started with model: %s", pr_.model_name.c_str());
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("detect_object", detect_object);
  ROS_INFO("[detect_object_server]: Ready to detect objects.");
  ros::spin();

  return 0;
}


