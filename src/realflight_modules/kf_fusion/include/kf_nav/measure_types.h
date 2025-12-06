#pragma once

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>

#include "kf_nav/commons.h"

enum class MeasureType : unsigned short {
  kInvalid = 0,
  kImu,
  kImuBatch,
  kOdometry,
  kGps
};
template <typename MsgType>
struct TypedMessage {
  static_assert(std::is_enum_v<MsgType>, "MsgType must be an enum type");

  virtual ~TypedMessage() {}

  double timestamp_s{0L};
  MsgType type;

 protected:
  TypedMessage() = default;
  TypedMessage(double timestamp_s_, MsgType type_)
      : timestamp_s(timestamp_s_), type(type_) {}
};

using MeasureBase = TypedMessage<MeasureType>;
POINTER_ALIAS_DEFINE(MeasureBase);

struct ImuMeas final : MeasureBase {
  static std::shared_ptr<ImuMeas> createPtr(double timestamp_s,
                                            const Eigen::Vector3d &acc,
                                            const Eigen::Vector3d &gyro) {
    return std::shared_ptr<ImuMeas>(new ImuMeas(timestamp_s, acc, gyro));
  }

  static std::shared_ptr<ImuMeas> createPtr(
      const sensor_msgs::ImuConstPtr &imu_msg) {
    std::shared_ptr<ImuMeas> imu{nullptr};
    if (imu_msg != nullptr) {
      Eigen::Vector3d acc_temp(imu_msg->linear_acceleration.x,
                               imu_msg->linear_acceleration.y,
                               imu_msg->linear_acceleration.z);
      Eigen::Vector3d gyro_temp(imu_msg->angular_velocity.x,
                                imu_msg->angular_velocity.y,
                                imu_msg->angular_velocity.z);
      imu.reset(
          new ImuMeas(imu_msg->header.stamp.toSec(), acc_temp, gyro_temp));

    } else {
      imu.reset(new ImuMeas());
      imu->type = MeasureType::kInvalid;
    }
    return imu;
  }
  Eigen::Vector3d acc_m_s2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyr_rad_s{Eigen::Vector3d::Zero()};

 protected:
  ImuMeas(double t_s, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro)
      : MeasureBase(t_s, MeasureType::kImu), acc_m_s2(acc), gyr_rad_s(gyro){};
  ImuMeas() = default;
};
POINTER_ALIAS_DEFINE(ImuMeas);

struct OdomMeas final : MeasureBase {
  static std::shared_ptr<OdomMeas> createPtr(double timestamp_s,
                                             const Eigen::Quaterniond &q_wb_,
                                             const Eigen::Vector3d &p_wb_,
                                             const Eigen::Vector3d &vel_w_) {
    return std::shared_ptr<OdomMeas>(
        new OdomMeas(timestamp_s, q_wb_, p_wb_, vel_w_));
  }

  static std::shared_ptr<OdomMeas> createPtr(
      const nav_msgs::OdometryConstPtr &odom_msg) {
    std::shared_ptr<OdomMeas> odom{nullptr};
    if (odom_msg != nullptr) {
      Eigen::Quaterniond q_temp(
          odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
          odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
      Eigen::Vector3d p_temp(odom_msg->pose.pose.position.x,
                             odom_msg->pose.pose.position.y,
                             odom_msg->pose.pose.position.z);
      Eigen::Vector3d vel_temp(odom_msg->twist.twist.linear.x,
                               odom_msg->twist.twist.linear.y,
                               odom_msg->twist.twist.linear.z);
      odom.reset(new OdomMeas(odom_msg->header.stamp.toSec(), q_temp, p_temp,
                              vel_temp));
      odom->setAccelBias(Eigen::Vector3d(odom_msg->twist.twist.angular.x,
                                         odom_msg->twist.twist.angular.y,
                                         odom_msg->twist.twist.angular.z));
    } else {
      odom.reset(new OdomMeas());
      odom->type = MeasureType::kInvalid;
    }
    return odom;
  }

  void setAccelBias(const Eigen::Vector3d &obv_bias_accel) {
    bias_accel = obv_bias_accel;
  }

  Eigen::Quaterniond q_w_b;
  Eigen::Vector3d p_w_b;
  Eigen::Vector3d vel_w;
  Eigen::Vector3d bias_accel;

 protected:
  OdomMeas(double t_s, const Eigen::Quaterniond &q_wb_,
           const Eigen::Vector3d &p_wb_, const Eigen::Vector3d &vel_w_)
      : MeasureBase(t_s, MeasureType::kOdometry),
        q_w_b(q_wb_),
        p_w_b(p_wb_),
        vel_w(vel_w_) {
    bias_accel.setZero();
  };
  OdomMeas() = default;
};
POINTER_ALIAS_DEFINE(OdomMeas)
