#include <ros/ros.h>
#include <unordered_map>
#include <visual_tools.h>
#include <trajectory_solver/msg_traj.h>
std::unique_ptr<TrajVisualNode> visual_tool_;
std::unordered_map<char, std_msgs::ColorRGBA> color_map_;
double max_a_;

void visualizeTraj(const mav_high_level_msgs::Trajectory::ConstPtr &traj_msg) {
  float dt = 0.05;
  MsgTrajectory traj(*traj_msg);
  vec_Vec3f poses, vels, accs, jrks;
  float total_time = traj.getTotalTime();
  float max_v = 0;
  Vec3f max_v_p;
  float max_a = 0;
  Vec3f max_a_p;

  Vec3f prev_pt(-1000, -1000, -1000);
  vec_Vec3f stop_ps;
  for (float t = dt; t <= total_time; t += dt) {
    Vec4 pos;
    Vec4 vel;
    Vec4 acc;
    Vec4 jrk;
    traj.evaluate(t, 0, pos);
    traj.evaluate(t, 1, vel);
    traj.evaluate(t, 2, acc);
    traj.evaluate(t, 3, jrk);

    Vec3f pt = pos.topRows<3>();
    if ((pt - prev_pt).norm() > 2e-1) {
      poses.push_back(pt);
      Vec3f v = vel.topRows<3>();
      vels.push_back(v);
      accs.push_back(acc.topRows<3>());
      jrks.push_back(jrk.topRows<3>());

      if (v.norm() > max_v) {
        max_v = v.norm();
        max_v_p = pt;
      }

      if (acc.topRows<3>().norm() > max_a) {
        max_a = acc.topRows<3>().norm();
        max_a_p = pt;
      }

      stop_ps.push_back(0.5 / max_a * v.norm() * v);
      prev_pt = pt;
    }
  }

  visual_tool_->publish_stop_pos(poses, stop_ps, traj_msg->header,
                                 color_map_[2]);

  visual_tool_->publish_pos(poses, traj_msg->header,
                            color_map_[traj_msg->mode]);
  visual_tool_->publish_vel(poses, vels, traj_msg->header);
  visual_tool_->publish_acc(poses, accs, traj_msg->header);
  visual_tool_->publish_text(max_v_p, max_v, "vel", traj_msg->header);
  visual_tool_->publish_text(max_a_p, max_a, "acc", traj_msg->header);
  // visual_tool_->publish_jrk(poses, jrks, traj_msg->header);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_vis");
  ros::NodeHandle nh("~");
  nh.param("max_a", max_a_, 3.0);

  ros::Subscriber traj_sub = nh.subscribe("traj", 1, visualizeTraj);

  visual_tool_.reset(new TrajVisualNode("vis"));
  visual_tool_->init(nh);

  color_map_.insert(std::make_pair(0, visual_tool_->MarkerPink));
  color_map_.insert(std::make_pair(1, visual_tool_->MarkerBlue));
  color_map_.insert(std::make_pair(2, visual_tool_->MarkerRed));
  color_map_.insert(std::make_pair(3, visual_tool_->MarkerRed));

  ros::spin();

  return 0;
}
