#pragma once

#include <Eigen/Dense>

#include "kf_nav/commons.h"
#include "sophus/so3.hpp"

class OriBase {
 public:
  static constexpr int LocalDim = LDim::Ori;
  OriBase() = default;
  explicit OriBase(const Sophus::SO3d& so3) : so3_(so3) {}
  explicit OriBase(const Eigen::Matrix3d& R) : so3_(R) {}
  explicit OriBase(const Eigen::Quaterniond& Q) : so3_(Q) {}
  explicit OriBase(const double* coeffs, int start_index)
      : start_index_(start_index) {
    Eigen::Map<const Eigen::Quaterniond> q_temp(coeffs);
    so3_ = Sophus::SO3d(q_temp.normalized());
  }
  constexpr int localSize() const { return LocalDim; }
  inline int startIndex() const { return start_index_; }
  inline void setConstant(bool is_constant) { is_constant_ = is_constant; }
  inline bool isConstant() const { return is_constant_; }
  inline void update(const double* delta_x) {
    Eigen::Map<const Eigen::Vector3d> delta_rot(delta_x);
    so3_ = so3_ * Sophus::SO3d::exp(delta_rot);
  }
  inline const Sophus::SO3d& so3() const { return so3_; }
  inline Sophus::SO3d& so3() { return so3_; }
  inline Eigen::Matrix3d rotationMat() { return so3_.matrix(); }

 protected:
  bool is_constant_{false};
  int start_index_{0};
  Sophus::SO3d so3_;
};

class VecBase {
 public:
  static constexpr int LocalDim = LDim::Vec;
  VecBase() = default;
  explicit VecBase(const Eigen::Vector3d& vec) : vec_(vec) {}
  explicit VecBase(const double* val, int start_index)
      : start_index_(start_index) {
    vec_ = Eigen::Map<const Eigen::Vector3d>(val);
  }
  constexpr int localSize() const { return LocalDim; }
  inline int startIndex() const { return start_index_; }
  inline void setConstant(bool is_constant) { is_constant_ = is_constant; }
  inline bool isConstant() const { return is_constant_; }
  inline void update(const double* delta_x) {
    Eigen::Map<const Eigen::Vector3d> delta_vec(delta_x);
    vec_ = vec_ + delta_vec;
  }
  inline const Eigen::Vector3d& vec() const { return vec_; }
  inline Eigen::Vector3d& vec() { return vec_; }

 protected:
  bool is_constant_{false};
  int start_index_{0};
  Eigen::Vector3d vec_;
};

class ImuOriState : public OriBase {
 public:
  static constexpr StateType Type = StateType::kImuOri;
  static constexpr int Order = StateOrder::ImuOri;

 public:
  ImuOriState() = default;
  explicit ImuOriState(const double* data, int start_index)
      : OriBase(data, start_index) {}
  constexpr int order() const { return Order; }
};

class ImuTvecState : public VecBase {
 public:
  static constexpr StateType Type = StateType::kImuTvec;
  static constexpr int Order = StateOrder::ImuTvec;
  ImuTvecState() = default;
  explicit ImuTvecState(const double* data, int start_index)
      : VecBase(data, start_index) {}
  constexpr int order() const { return Order; }
};

class ImuVelState : public VecBase {
 public:
  static constexpr StateType Type = StateType::kImuVel;
  static constexpr int Order = StateOrder::ImuVel;

 public:
  ImuVelState() = default;
  explicit ImuVelState(const double* data, int start_index)
      : VecBase(data, start_index) {}
  constexpr int order() const { return Order; }
};

class BiasGyroState : public VecBase {
 public:
  static constexpr StateType Type = StateType::kImuBiasGyro;
  static constexpr int Order = StateOrder::ImuBiasGyro;

 public:
  BiasGyroState() = default;
  explicit BiasGyroState(const double* data, int start_index)
      : VecBase(data, start_index) {}
  constexpr int order() const { return Order; }
};

class BiasAccelState : public VecBase {
 public:
  static constexpr StateType Type = StateType::kImuBiasAccel;
  static constexpr int Order = StateOrder::ImuBiasAccel;

 public:
  BiasAccelState() = default;
  explicit BiasAccelState(const double* data, int start_index)
      : VecBase(data, start_index) {}
  constexpr int order() const { return Order; }
};

class ExtOdomOriState : public OriBase {
 public:
  static constexpr StateType Type = StateType::kExtOdomOri;
  static constexpr int Order = StateOrder::ExtOdomOri;

 public:
  ExtOdomOriState() = default;
  explicit ExtOdomOriState(const double* data, int start_index)
      : OriBase(data, start_index) {}
  constexpr int order() const { return Order; }
};

class ExtOdomTvecState : public VecBase {
 public:
  static constexpr StateType Type = StateType::kExtOdomTvec;
  static constexpr int Order = StateOrder::ExtOdomTvec;

 public:
  ExtOdomTvecState() = default;
  explicit ExtOdomTvecState(const double* data, int start_index)
      : VecBase(data, start_index) {}
  constexpr int order() const { return Order; }
};
