#include <ros/ros.h>
#include <string>
#include <fstream>
#include <unistd.h>
#include <sys/wait.h>

int main(int argc, char **argv)
{
  // ROS_INFO("argv[0] = %s",argv[0]);
  ros::init(argc, argv, "object_detector");
  
  ros::NodeHandle nh("object_detector");

  // nh.param("overlap",    params.overlap,    0.5);
  // nh.param("threshold",  params.threshold, -1.0);
  // nh.param("interval",   params.interval,   5);
  // nh.param("padding",    params.padding,    6);
  // nh.param<std::string>("model_name", pr_.model_name, "object");

  std::string model_dir;

  if (!nh.getParam("model_directory", model_dir)) {
    ROS_ERROR("Parameter model_directory not set");
    return 1;
  }

  // std::cout << "model dir = " << model_dir << std::endl;

  std::string cam_topic;
  if (!nh.getParam("cam_topic", cam_topic)) {
      ROS_ERROR("Parameter cam_topic not set!");
      return 1;
  }

  std::vector<std::string> model_names;
  nh.getParam("model_names", model_names);

  // std::string launch_command = "roslaunch ffld_ros obj_detect_node.launch";
  std::string launch_command = "rosrun ffld_ros detect_object_node";
  launch_command += " _overlap:=0.4";
  launch_command += " _model_directory:=" + model_dir;
  launch_command += " _cam_topic:=" + cam_topic;
  launch_command += " _model:="; // append model name at end

  std::vector<pid_t> children;

  for (size_t i = 0; i < model_names.size(); ++i) {

    std::string model_dir_arg = "_model_directory:=" + model_dir;
    std::string cam_topic_arg = "_cam_topic:=" + cam_topic;
    std::string model_arg = "_model:=" + model_names[i];

    char model_dir_cstr[128];
    char cam_cstr[128];
    char model_cstr[128];

    strcpy(model_dir_cstr, model_dir_arg.c_str());
    strcpy(cam_cstr, cam_topic_arg.c_str());
    strcpy(model_cstr, model_arg.c_str());

    char *new_args[10];

    char **next = new_args;

    *next++ = "rosrun";
    *next++ = "ffld_ros";
    *next++ = "detect_object_node";
    *next++ = "_overlap:=0.4";
    *next++ = model_dir_cstr;
    *next++ = cam_cstr;
    *next++ = model_cstr;
    *next++ = 0;

    std::string cmd = launch_command + model_names[i];// + "&";

    ROS_INFO_STREAM("Launching \"" << cmd << "\"");

    pid_t parent = getpid();
    pid_t pid = fork();

    if (pid == -1) {
      ROS_ERROR("Failed to fork object detector process");
      exit(1);
    } else if (pid > 0) {
      children.push_back(pid);
    } else {
      execvp(new_args[0], new_args);
      ROS_ERROR("Object detector execution failed");
      exit(1);
    }
  }

  ros::spin();

  // there should be a nicer way to do this
  // poor kids
  for (pid_t child : children) {
    kill(child, 9);
  }

  // int status;
  // for (pid_t child : children) {
  //   waitpid(child, &status, 0);
  // }

  return 0;
}


