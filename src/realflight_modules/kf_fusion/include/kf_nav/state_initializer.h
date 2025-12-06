#pragma once

#include <deque>

#include "kf_nav/commons.h"
#include "kf_nav/measure_types.h"
#include "kf_nav/parameter.hpp"

class StateInitializer {
 public:
  struct StateInit {
    Eigen::Quaterniond q_w_i;
    Eigen::Vector3d p_w_i;
    Eigen::Vector3d vel_w;
    Eigen::Vector3d bias_gyro;
    Eigen::Vector3d bias_accel;
    Eigen::Quaterniond q_i_b;
    Eigen::Vector3d p_i_b;

    double timestamp_s{0};
    Eigen::Vector3d gyro_init;
    Eigen::Vector3d accel_init;
    StateInit() {
      q_w_i.setIdentity();
      p_w_i.setZero();
      vel_w.setZero();
      bias_gyro.setZero();
      bias_accel.setZero();
      q_i_b.setIdentity();
      p_i_b.setZero();
      gyro_init.setZero();
      accel_init.setZero();
    }
  };

 public:
  StateInitializer() {
    static_check_dura_s_ =
        ParamReader::getInstance().getDouble("static_check_dura_s", 5.0);
    thresh_gyro_avg_ =
        ParamReader::getInstance().getDouble("thresh_gyro_avg", 0.01);
    thresh_accel_avg_ =
        ParamReader::getInstance().getDouble("thresh_accel_avg", 0.2);
    thresh_gyro_var_ =
        ParamReader::getInstance().getDouble("thresh_gyro_var", 0.002);
    thresh_accel_var_ =
        ParamReader::getInstance().getDouble("thresh_accel_var", 0.1);

    bool use_motion_capture =
        ParamReader::getInstance().getBool("use_motion_capture", false);
    Eigen::Matrix4d T_i_b;
    if (use_motion_capture) {
      T_i_b = ParamReader::getInstance().getFixedMatrix<double, 4, 4>(
          "T_imu_mbody", Eigen::Matrix4d::Identity());
    } else {
      T_i_b = ParamReader::getInstance().getFixedMatrix<double, 4, 4>(
          "T_imu_body", Eigen::Matrix4d::Identity());
    }
    Eigen::Matrix3d R_i_b = T_i_b.topLeftCorner(3, 3);
    state_.q_i_b = Eigen::Quaterniond(R_i_b);
    state_.p_i_b = T_i_b.topRightCorner(3, 1);

    state_.bias_accel = ParamReader::getInstance().getFixedVector<double, 3>(
        "bias_accel_init", Eigen::Vector3d::Zero());
  }
  bool initState(const MeasureBasePtr& meas);
  void clearBuffer();
  inline StateInit getInitialState() { return state_; }

 protected:
  double getImuQueueDura();

  bool initGyroBias();
  bool initByOdom();

 protected:
  double static_check_dura_s_{5.0};
  double thresh_gyro_avg_{0.01};
  double thresh_accel_avg_{0.2};
  double thresh_gyro_var_{0.002};
  double thresh_accel_var_{0.1};
  bool is_bias_init_{false};
  bool is_state_init_{false};
  StateInit state_;
  std::deque<ImuMeasPtr> imu_que_;
  std::deque<OdomMeasPtr> odom_que_;
};