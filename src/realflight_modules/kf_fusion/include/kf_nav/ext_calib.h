#pragma once

#include <ros/ros.h>

#include <memory>
#include <optional>
#include <thread>

#include "eigen3/Eigen/Dense"
#include "kf_nav/parameter.hpp"
#include "kf_nav/safe_queue.h"
#include "nav_msgs/Odometry.h"

struct OdomPose {
  double timestamp;
  Eigen::Quaterniond q_w_i;
  Eigen::Vector3d p_w_i;
  static std::shared_ptr<OdomPose> create(double t, const Eigen::Quaterniond& q,
                                          const Eigen::Vector3d& p) {
    std::shared_ptr<OdomPose> out(new OdomPose);
    out->timestamp = t;
    out->q_w_i = q;
    out->p_w_i = p;
    return out;
  }
};
typedef std::shared_ptr<OdomPose> OdomPosePtr;

class ExtCalib {
 public:
  enum class State {
    ready = 0,
    dataCollection = 1,
    solving = 2,
    termination = 3
  };
  struct Config {
    bool calib_ori{false};
    std::string vins_topic;
    std::string mc_topic;
    Eigen::Quaterniond ext_q_init;

    void loadConfig() {
      calib_ori = ParamReader::getInstance().getBool("estimate_ext_rot", false);
      vins_topic =
          ParamReader::getInstance().getString("odom_topic", "/odometry");
      mc_topic = ParamReader::getInstance().getString("motion_capture_topic",
                                                      "/mc_odometry");
      Eigen::Matrix4d T_i_b;
      T_i_b = ParamReader::getInstance().getFixedMatrix<double, 4, 4>(
          "T_imu_mbody", Eigen::Matrix4d::Identity());
      Eigen::Matrix3d R_i_b = T_i_b.topLeftCorner(3, 3);
      ext_q_init = Eigen::Quaterniond(R_i_b);
    }
  };

 public:
  void init(ros::NodeHandle& nh);
  void processData();

 protected:
  void receive_vins_msg(const nav_msgs::OdometryPtr& msg);
  void receive_mc_msg(const nav_msgs::OdometryPtr& msg);
  bool judgeMotion(const OdomPosePtr& last_pose, const OdomPosePtr& cur_pose);
  std::optional<OdomPosePtr> interpolAndPopMcPose(double t);
  bool solveExtPose();

 protected:
  ros::Subscriber sub_vins_odom_;
  ros::Subscriber sub_mc_odom_;

  std::thread thrd_hdl_;

  Config config_;
  State cur_state_{State::ready};
  int sequal_invalid_pose_cnt_ = 0;
  Eigen::Quaterniond ext_q_ib_;
  Eigen::Vector3d ext_p_ib_;

  ThreadsafeQueue<OdomPosePtr> vins_msg_queue_;
  ThreadsafeQueue<OdomPosePtr> mc_msg_queue_;
  std::vector<OdomPosePtr> vins_solve_pose_;
  std::vector<OdomPosePtr> mc_solve_pose_;
};