#pragma once

#include "kf_nav/commons.h"
#include "kf_nav/measure_types.h"
#include "kf_nav/parameter.hpp"
#include "kf_nav/state_initializer.h"
#include "kf_nav/state_manager.h"

using DUMatType = Eigen::Matrix<double, total_local_size, total_local_size>;

struct DelayUpdateInfo {
  DUMatType d_si_d_sj;
  Eigen::Matrix3d deltaR;
  Eigen::Vector3d deltaP;
  Eigen::Vector3d deltaV;
  double deltaT;
  DelayUpdateInfo() {
    d_si_d_sj.setIdentity();
    deltaR.setIdentity();
    deltaP.setZero();
    deltaV.setZero();
    deltaT = 0;
  }
};

class KfCoordinator {
 public:
  struct Config {
    double accel_noise_std = 0.1;
    double gyro_noise_std = 0.02;
    double accel_walk_std = 5e-4;
    double gyro_walk_std = 1e-4;

    double imu_ori_init_cov = 4e-5;
    double imu_yaw_init_cov = 1e-5;
    double imu_posi_init_cov = 1e-5;
    double imu_vel_init_cov = 0.09;
    double gyro_bias_init_cov = 1e-7;
    double acc_bias_init_cov = 1e-2;
    double ext_ori_init_cov = 1e-6;
    double ext_posi_init_cov = 1e-4;

    bool use_motion_capture = false;
    bool estimate_ext_rot = false;
    bool estimate_ext_tvec = true;
    bool enable_merge_vins_bias = false;

    double imu_que_dura_length_{1.0};

    Eigen::Vector3d obv_odom_rot_cov;
    Eigen::Vector3d obv_odom_posi_cov;
    Eigen::Vector3d obv_odom_vel_cov;
    Eigen::Vector3d obv_vins_bias_cov;

    Eigen::Vector3d obv_mc_rot_cov;
    Eigen::Vector3d obv_mc_posi_cov;
    Eigen::Vector3d obv_mc_vel_cov;

    Eigen::Vector3d obv_zupt_gyro_cov;
    Eigen::Vector3d obv_zupt_accel_cov;
    Eigen::Vector3d obv_zupt_vel_cov;

    void loadConfig() {
      accel_noise_std =
          ParamReader::getInstance().getDouble("accel_noise_std", 0.1);
      gyro_noise_std =
          ParamReader::getInstance().getDouble("gyro_noise_std", 0.02);
      accel_walk_std =
          ParamReader::getInstance().getDouble("accel_walk_std", 1e-3);
      gyro_walk_std =
          ParamReader::getInstance().getDouble("gyro_walk_std", 1e-4);

      imu_ori_init_cov =
          ParamReader::getInstance().getDouble("imu_ori_init_cov", 4e-5);
      imu_yaw_init_cov =
          ParamReader::getInstance().getDouble("imu_yaw_init_cov", 1e-5);
      imu_posi_init_cov =
          ParamReader::getInstance().getDouble("imu_posi_init_cov", 1e-5);
      imu_vel_init_cov =
          ParamReader::getInstance().getDouble("imu_vel_init_cov", 0.09);
      gyro_bias_init_cov =
          ParamReader::getInstance().getDouble("gyro_bias_init_cov", 1e-7);
      acc_bias_init_cov =
          ParamReader::getInstance().getDouble("acc_bias_init_cov", 1e-2);
      ext_ori_init_cov =
          ParamReader::getInstance().getDouble("ext_ori_init_cov", 1e-6);
      ext_posi_init_cov =
          ParamReader::getInstance().getDouble("ext_posi_init_cov", 1e-4);

      use_motion_capture =
          ParamReader::getInstance().getBool("use_motion_capture", false);
      estimate_ext_rot =
          ParamReader::getInstance().getBool("estimate_ext_rot", false);
      estimate_ext_tvec =
          ParamReader::getInstance().getBool("estimate_ext_tvec", true);
      enable_merge_vins_bias =
          ParamReader::getInstance().getBool("enable_merge_vins_bias", false);

      imu_que_dura_length_ =
          ParamReader::getInstance().getDouble("imu_que_dura_length", 1.0);

      obv_odom_rot_cov = ParamReader::getInstance().getFixedVector<double, 3>(
          "obv_odom_rot_cov", Eigen::Vector3d::Identity());
      obv_odom_posi_cov = ParamReader::getInstance().getFixedVector<double, 3>(
          "obv_odom_posi_cov", Eigen::Vector3d::Identity());
      obv_odom_vel_cov = ParamReader::getInstance().getFixedVector<double, 3>(
          "obv_odom_vel_cov", Eigen::Vector3d::Identity());
      obv_vins_bias_cov = ParamReader::getInstance().getFixedVector<double, 3>(
          "obv_vins_bias_cov", Eigen::Vector3d::Identity());

      obv_mc_rot_cov = ParamReader::getInstance().getFixedVector<double, 3>(
          "obv_mc_rot_cov", Eigen::Vector3d::Identity());
      obv_mc_posi_cov = ParamReader::getInstance().getFixedVector<double, 3>(
          "obv_mc_posi_cov", Eigen::Vector3d::Identity());
      obv_mc_vel_cov = ParamReader::getInstance().getFixedVector<double, 3>(
          "obv_mc_vel_cov", Eigen::Vector3d::Identity());

      obv_zupt_gyro_cov = ParamReader::getInstance().getFixedVector<double, 3>(
          "obv_zupt_gyro_cov", Eigen::Vector3d::Identity());
      obv_zupt_accel_cov = ParamReader::getInstance().getFixedVector<double, 3>(
          "obv_zupt_gyro_cov", Eigen::Vector3d::Identity());
      obv_zupt_vel_cov = ParamReader::getInstance().getFixedVector<double, 3>(
          "obv_zupt_vel_cov", Eigen::Vector3d::Identity());
    }
  };
  Config kf_config;

 public:
  KfCoordinator() {
    kf_config.loadConfig();
    state_cov_.resize(total_local_size, total_local_size);
    state_cov_.setZero();
    state_cov_.diagonal().setConstant(1e-12);
    imu_noise_cov_.setZero();
    imu_noise_cov_.block<3, 3>(0, 0) =
        (kf_config.accel_noise_std * kf_config.accel_noise_std) *
        Eigen::Matrix3d::Identity();
    imu_noise_cov_.block<3, 3>(3, 3) =
        (kf_config.gyro_noise_std * kf_config.gyro_noise_std) *
        Eigen::Matrix3d::Identity();
    imu_noise_cov_.block<3, 3>(6, 6) =
        (kf_config.accel_noise_std * kf_config.accel_noise_std) *
        Eigen::Matrix3d::Identity();
    imu_noise_cov_.block<3, 3>(9, 9) =
        (kf_config.gyro_noise_std * kf_config.gyro_noise_std) *
        Eigen::Matrix3d::Identity();
    imu_noise_cov_.block<3, 3>(12, 12) =
        (kf_config.accel_walk_std * kf_config.accel_walk_std) *
        Eigen::Matrix3d::Identity();
    imu_noise_cov_.block<3, 3>(15, 15) =
        (kf_config.gyro_walk_std * kf_config.gyro_walk_std) *
        Eigen::Matrix3d::Identity();
  }

  void insertStateWithCov(UniID state_id, const double* data,
                          StateType state_type,
                          const Eigen::MatrixXd& init_cov);
  void insertConstState(UniID state_id, const double* data,
                        StateType state_type);
  void init(const StateInitializer::StateInit& state_init);
  inline bool isInit() { return is_init_; }

  void processImuMeas(const ImuMeasPtr& imu_meas);
  void imuStatePropagate(double cur_imu_time, const Eigen::Vector3d& cur_gyro,
                         const Eigen::Vector3d& cur_accel);
  void updateWithOdomMeas(const OdomMeasPtr& odom_meas);
  void updateWithZUPT(const ImuMeasPtr& imu_meas);
  const StateManager& getStateManagerRef() const { return state_manager_; }
  StateManager& getStateManagerRef() { return state_manager_; }
  Eigen::Vector3d getImuOriCov();
  Eigen::Vector3d getImuTvecCov();
  Eigen::Vector3d getImuVelCov();

 protected:
  double getImuQueueDura();
  void addResidualJacobian(const Eigen::VectorXd& residual,
                           const std::vector<Eigen::MatrixXd>& jacobians,
                           const std::vector<IdxSizePair>& id_size_pairs,
                           const DUMatType& du_jacobian);
  DelayUpdateInfo getDelayUpdateInfo(double delay_update_time);
  void solveAndUpdate();

  StateManager state_manager_;

  bool is_init_{false};

  double last_imu_time_{0};
  Eigen::Vector3d last_gyro_;
  Eigen::Vector3d last_accel_;

  Eigen::MatrixXd state_cov_;
  Eigen::Matrix<double, 18, 18> imu_noise_cov_;

  std::vector<Eigen::VectorXd> obv_res_;
  std::vector<Eigen::MatrixXd> obv_jacobian_;
  Eigen::MatrixXd obv_R;

  std::deque<ImuMeasPtr> imu_que_;
};
