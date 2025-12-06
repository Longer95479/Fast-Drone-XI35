#pragma once

#include <Eigen/Dense>
#include <deque>

#include "kf_nav/measure_types.h"
#include "sophus/so3.hpp"

class ImuHelper {
 public:
  struct PreintegrationRes {
    PreintegrationRes() {
      deltaR.setIdentity();
      deltaP.setZero();
      deltaV.setZero();
    }
    PreintegrationRes(const Eigen::Matrix3d &deltaR_,
                      const Eigen::Vector3d &deltaP_,
                      const Eigen::Vector3d &deltaV_)
        : deltaR(deltaR_), deltaP(deltaP_), deltaV(deltaV_) {}
    Eigen::Matrix3d deltaR;
    Eigen::Vector3d deltaP;
    Eigen::Vector3d deltaV;
  };

  template <typename ContainerType>
  static PreintegrationRes getPreintegThroughImuQue(
      const ContainerType &imu_que, const Eigen::Vector3d &bias_gyro,
      const Eigen::Vector3d &bias_accel) {
    PreintegrationRes preintg_res;
    if (imu_que.size() < 2) return preintg_res;
    auto last_gyro = imu_que[0]->gyr_rad_s;
    auto last_accel = imu_que[0]->acc_m_s2;
    double last_time = imu_que[0]->timestamp_s;
    for (int i = 1; i < imu_que.size(); i++) {
      const auto &cur_gyro = imu_que[i]->gyr_rad_s;
      const auto &cur_accel = imu_que[i]->acc_m_s2;
      double dt = imu_que[i]->timestamp_s - last_time;
      Eigen::Vector3d gyro_mid = 0.5 * (last_gyro + cur_gyro) - bias_gyro;
      Eigen::Vector3d accel_0 = preintg_res.deltaR * (last_accel - bias_accel);
      preintg_res.deltaR *= Sophus::SO3d::exp(gyro_mid * dt).matrix();
      Eigen::Vector3d accel_1 = preintg_res.deltaR * (cur_accel - bias_accel);
      Eigen::Vector3d accel_mid = 0.5 * (accel_0 + accel_1);
      preintg_res.deltaP += preintg_res.deltaV * dt + 0.5 * accel_mid * dt * dt;
      preintg_res.deltaV += accel_mid * dt;
      last_gyro = cur_gyro;
      last_accel = cur_accel;
      last_time = imu_que[i]->timestamp_s;
    }
    return preintg_res;
  }

  static ImuMeasPtr imuLinearInterPolation(const ImuMeasPtr &imu0,
                                           const ImuMeasPtr &imu1,
                                           double t_intp) {
    double coffe_0 =
        (imu1->timestamp_s - t_intp) / (imu1->timestamp_s - imu0->timestamp_s);
    double coffe_1 =
        (t_intp - imu0->timestamp_s) / (imu1->timestamp_s - imu0->timestamp_s);
    auto dummy_gyro = coffe_0 * imu0->gyr_rad_s + coffe_1 * imu1->gyr_rad_s;
    auto dummy_accel = coffe_0 * imu0->acc_m_s2 + coffe_1 * imu1->acc_m_s2;
    return ImuMeas::createPtr(t_intp, dummy_accel, dummy_gyro);
  }
};

class StaticCheck {
 public:
  void inputImu(const ImuMeasPtr &imu) {
    if (!imu_que_.empty() && imu->timestamp_s < imu_que_.back()->timestamp_s)
      return;
    imu_que_.push_back(imu);
    is_static_ = checkStatic();
    while ((imu_que_.back()->timestamp_s - imu_que_.front()->timestamp_s) >
           static_check_dura_s) {
      imu_que_.pop_front();
    }
  }
  bool checkStatic() {
    if (imu_que_.size() < 10) return false;
    Eigen::Vector3d gyro_avg{0, 0, 0};
    Eigen::Vector3d accel_avg{0, 0, 0};
    for (auto &imu_ptr : imu_que_) {
      gyro_avg += imu_ptr->gyr_rad_s;
      accel_avg += imu_ptr->acc_m_s2;
    }
    gyro_avg *= (1. / imu_que_.size());
    accel_avg *= (1. / imu_que_.size());

    double gyro_var = 0.;
    double accel_var = 0.;
    for (auto &imu_ptr : imu_que_) {
      gyro_var +=
          (imu_ptr->gyr_rad_s - gyro_avg).dot(imu_ptr->gyr_rad_s - gyro_avg);
      accel_var +=
          (imu_ptr->acc_m_s2 - accel_avg).dot(imu_ptr->acc_m_s2 - accel_avg);
    }
    gyro_var /= (-1. + imu_que_.size());
    accel_var /= (-1. + imu_que_.size());

    double gyro_avg_norm = gyro_avg.norm();
    double accel_avg_norm = std::fabs(accel_avg.norm() - kGravityMag);
    if (gyro_avg_norm < thresh_gyro_avg_ &&
        accel_avg_norm < thresh_accel_avg_ && gyro_var < thresh_gyro_var_ &&
        accel_var < thresh_accel_var_) {
      return true;
    }
    return false;
  }
  inline bool isStatic() { return is_static_; }

 protected:
  double static_check_dura_s{1.0};
  double thresh_gyro_avg_{0.01};
  double thresh_accel_avg_{0.2};
  double thresh_gyro_var_{0.002};
  double thresh_accel_var_{0.1};

  bool is_static_{false};
  std::deque<ImuMeasPtr> imu_que_;
};
