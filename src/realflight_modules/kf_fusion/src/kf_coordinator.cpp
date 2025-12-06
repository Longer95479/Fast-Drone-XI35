#include "kf_nav/kf_coordinator.h"

#include "kf_nav/imu_helper.h"

template <typename T>
Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1>& vec) {
  return (Eigen::Matrix<T, 3, 3>() << T(0), -vec(2), vec(1), vec(2), T(0),
          -vec(0), -vec(1), vec(0), T(0))
      .finished();
}

Eigen::Vector3d KfCoordinator::getImuOriCov() {
  return state_cov_.block<3, 3>(0, 0).diagonal().cwiseSqrt();
}

Eigen::Vector3d KfCoordinator::getImuTvecCov() {
  return state_cov_.block<3, 3>(3, 3).diagonal().cwiseSqrt();
}

Eigen::Vector3d KfCoordinator::getImuVelCov() {
  return state_cov_.block<3, 3>(6, 6).diagonal().cwiseSqrt();
}

double KfCoordinator::getImuQueueDura() {
  if (imu_que_.empty()) return 0;
  return (imu_que_.back()->timestamp_s - imu_que_.front()->timestamp_s);
}

void KfCoordinator::insertStateWithCov(UniID state_id, const double* data,
                                       StateType state_type,
                                       const Eigen::MatrixXd& init_cov) {
  auto idx_size_opt =
      state_manager_.insertState(state_id, state_type, data, false);
  if (idx_size_opt.has_value()) {
    auto [start_index, local_size] = idx_size_opt.value();
    if (init_cov.rows() != local_size || init_cov.cols() != local_size) {
      std::cerr << "init_cov's Dim is wrong! target state is"
                << static_cast<int>(state_type) << std::endl;
      return;
    }
    state_cov_.block(start_index, start_index, local_size, local_size) =
        init_cov;
    ROS_DEBUG("InserState: state type %d, start index %d, local size %d.",
              state_type, start_index, local_size);
  } else {
    std::cerr << "insert state wrong!" << std::endl;
  }
  return;
}

void KfCoordinator::insertConstState(UniID state_id, const double* data,
                                     StateType state_type) {
  auto idx_size_opt =
      state_manager_.insertState(state_id, state_type, data, true);
  if (!idx_size_opt.has_value()) {
    std::cerr << "insert state wrong!" << std::endl;
    return;
  }
  auto [start_index, local_size] = idx_size_opt.value();
  ROS_DEBUG("InserConstState: state type %d, start index %d, local size %d.",
            state_type, start_index, local_size);
  return;
}

void KfCoordinator::init(const StateInitializer::StateInit& state_init) {
  last_imu_time_ = state_init.timestamp_s;
  last_gyro_ = state_init.gyro_init;
  last_accel_ = state_init.accel_init;
  // insert imu states
  Eigen::Matrix3d R_cov_mat = Eigen::Matrix3d::Identity();
  R_cov_mat.diagonal().setConstant(kf_config.imu_ori_init_cov);
  insertStateWithCov(getImuOriUid(), state_init.q_w_i.coeffs().data(),
                     StateType::kImuOri, R_cov_mat);

  Eigen::Matrix3d p_cov_mat = Eigen::Matrix3d::Identity();
  p_cov_mat.diagonal().setConstant(kf_config.imu_posi_init_cov);
  insertStateWithCov(getImuTvecUid(), state_init.p_w_i.data(),
                     StateType::kImuTvec, p_cov_mat);

  Eigen::Matrix3d vel_cov_mat = Eigen::Matrix3d::Identity();
  vel_cov_mat.diagonal().setConstant(kf_config.imu_vel_init_cov);
  insertStateWithCov(getImuVelUid(), state_init.vel_w.data(),
                     StateType::kImuVel, vel_cov_mat);

  Eigen::Matrix3d bg_cov_mat = Eigen::Matrix3d::Identity();
  bg_cov_mat.diagonal().setConstant(kf_config.gyro_bias_init_cov);
  insertStateWithCov(getImuBiasGyroUid(), state_init.bias_gyro.data(),
                     StateType::kImuBiasGyro, bg_cov_mat);

  Eigen::Matrix3d ba_cov_mat = Eigen::Matrix3d::Identity();
  ba_cov_mat.diagonal().setConstant(kf_config.acc_bias_init_cov);
  insertStateWithCov(getImuBiasAccelUid(), state_init.bias_accel.data(),
                     StateType::kImuBiasAccel, ba_cov_mat);

  if (kf_config.estimate_ext_rot) {
    Eigen::Matrix3d ext_rot_cov_mat = Eigen::Matrix3d::Identity();
    ext_rot_cov_mat.diagonal().setConstant(kf_config.ext_ori_init_cov);
    insertStateWithCov(getExtOdomOriUid(), state_init.q_i_b.coeffs().data(),
                       StateType::kExtOdomOri, ext_rot_cov_mat);
  } else {
    insertConstState(getExtOdomOriUid(), state_init.q_i_b.coeffs().data(),
                     StateType::kExtOdomOri);
  }
  if (kf_config.estimate_ext_tvec) {
    Eigen::Matrix3d ext_tvec_cov_mat = Eigen::Matrix3d::Identity();
    ext_tvec_cov_mat.diagonal().setConstant(kf_config.ext_posi_init_cov);
    insertStateWithCov(getExtOdomTvecUid(), state_init.p_i_b.data(),
                       StateType::kExtOdomTvec, ext_tvec_cov_mat);
  } else {
    insertConstState(getExtOdomTvecUid(), state_init.p_i_b.data(),
                     StateType::kExtOdomTvec);
  }
  is_init_ = true;
  // std::cout << "initial state_cov_: " << std::endl << state_cov_ <<
  // std::endl;
}

void KfCoordinator::processImuMeas(const ImuMeasPtr& imu_meas) {
  imu_que_.push_back(imu_meas);
  while (getImuQueueDura() > kf_config.imu_que_dura_length_) {
    imu_que_.pop_front();
  }
  imuStatePropagate(imu_meas->timestamp_s, imu_meas->gyr_rad_s,
                    imu_meas->acc_m_s2);
}
void KfCoordinator::imuStatePropagate(double cur_imu_time,
                                      const Eigen::Vector3d& cur_gyro,
                                      const Eigen::Vector3d& cur_accel) {
  if (cur_imu_time < last_imu_time_) return;
  double dt = cur_imu_time - last_imu_time_;
  const Eigen::Vector3d gravity(0., 0., -kGravityMag);
  Sophus::SO3d& state_so3 = state_manager_.getImuOriState();
  Eigen::Matrix3d state_R0 = state_so3.matrix();
  Eigen::Vector3d& state_tvec = state_manager_.getImuTvecState();
  Eigen::Vector3d& state_vel = state_manager_.getImuVelState();
  Eigen::Vector3d& state_bg = state_manager_.getBiasGyroState();
  Eigen::Vector3d& state_ba = state_manager_.getBiasAccelState();

  Eigen::Vector3d gyro_mid = 0.5 * (last_gyro_ + cur_gyro) - state_bg;
  Eigen::Vector3d accel_w_0 = state_R0 * (last_accel_ - state_ba) + gravity;
  state_so3 = state_so3 * Sophus::SO3d::exp(gyro_mid * dt);
  Eigen::Matrix3d state_R1 = state_so3.matrix();
  Eigen::Vector3d accel_w_1 = state_R1 * (cur_accel - state_ba) + gravity;
  Eigen::Vector3d accel_w_mid = 0.5 * (accel_w_0 + accel_w_1);
  state_tvec += state_vel * dt + 0.5 * accel_w_mid * dt * dt;
  state_vel += accel_w_mid * dt;

  Eigen::Vector3d a_0_x = last_accel_ - state_ba;
  Eigen::Vector3d a_1_x = cur_accel - state_ba;
  Eigen::Matrix3d R_w_x = skew(gyro_mid);
  Eigen::Matrix3d R_a_0_x = skew(a_0_x);
  Eigen::Matrix3d R_a_1_x = skew(a_1_x);

  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(15, 15);
  F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() - R_w_x * dt;
  F.block<3, 3>(0, 9) = -1.0 * Eigen::MatrixXd::Identity(3, 3) * dt;
  F.block<3, 3>(3, 0) = -0.25 * state_R0 * R_a_0_x * dt * dt +
                        -0.25 * state_R1 * R_a_1_x *
                            (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt *
                            dt;
  F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  F.block<3, 3>(3, 6) = 1.0 * Eigen::MatrixXd::Identity(3, 3) * dt;
  F.block<3, 3>(3, 9) = -0.25 * state_R1 * R_a_1_x * dt * dt * -dt;
  F.block<3, 3>(3, 12) = -0.25 * (state_R0 + state_R1) * dt * dt;
  F.block<3, 3>(6, 0) = -0.5 * state_R0 * R_a_0_x * dt +
                        -0.5 * state_R1 * R_a_1_x *
                            (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt;
  F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
  F.block<3, 3>(6, 9) = -0.5 * state_R1 * R_a_1_x * dt * -dt;
  F.block<3, 3>(6, 12) = -0.5 * (state_R0 + state_R1) * dt;
  F.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
  F.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();

  Eigen::MatrixXd V = Eigen::MatrixXd::Zero(15, 18);
  V.block<3, 3>(0, 3) = 0.5 * Eigen::MatrixXd::Identity(3, 3) * dt;
  V.block<3, 3>(0, 9) = 0.5 * Eigen::MatrixXd::Identity(3, 3) * dt;
  V.block<3, 3>(3, 0) = 0.25 * state_R0 * dt * dt;
  V.block<3, 3>(3, 3) = 0.25 * -state_R1 * R_a_1_x * dt * dt * 0.5 * dt;
  V.block<3, 3>(3, 6) = 0.25 * state_R1 * dt * dt;
  V.block<3, 3>(3, 9) = V.block<3, 3>(3, 3);
  V.block<3, 3>(6, 0) = 0.5 * state_R0 * dt;
  V.block<3, 3>(6, 3) = 0.5 * -state_R1 * R_a_1_x * dt * 0.5 * dt;
  V.block<3, 3>(6, 6) = 0.5 * state_R1 * dt;
  V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);
  V.block<3, 3>(9, 15) = Eigen::MatrixXd::Identity(3, 3) * dt;
  V.block<3, 3>(12, 12) = Eigen::MatrixXd::Identity(3, 3) * dt;

  state_cov_.block<15, 15>(0, 0) =
      F * state_cov_.block<15, 15>(0, 0) * F.transpose() +
      V * imu_noise_cov_ * V.transpose();

  last_imu_time_ = cur_imu_time;
  last_gyro_ = cur_gyro;
  last_accel_ = cur_accel;
}

void KfCoordinator::addResidualJacobian(
    const Eigen::VectorXd& residual,
    const std::vector<Eigen::MatrixXd>& jacobians,
    const std::vector<IdxSizePair>& id_size_pairs,
    const DUMatType& du_jacobian) {
  int res_dim = static_cast<int>(residual.size());
  Eigen::MatrixXd jacobian_aug =
      Eigen::MatrixXd::Zero(res_dim, total_local_size);
  assert(jacobians.size() == id_size_pairs.size());
  for (int i = 0; i < jacobians.size(); i++) {
    auto [start_idx, local_size] = id_size_pairs[i];
    jacobian_aug.block(0, start_idx, res_dim, local_size) = jacobians[i];
  }
  jacobian_aug *= du_jacobian;
  obv_res_.push_back(residual);
  obv_jacobian_.push_back(jacobian_aug);
}

DelayUpdateInfo KfCoordinator::getDelayUpdateInfo(double delay_update_time) {
  DelayUpdateInfo delay_update_info;
  if (imu_que_.empty() || delay_update_time >= imu_que_.back()->timestamp_s ||
      delay_update_time < imu_que_.front()->timestamp_s) {
    return delay_update_info;
  }
  std::deque<ImuMeasPtr> preintg_que;
  for (auto it = imu_que_.rbegin(); it != imu_que_.rend(); it++) {
    if ((*it)->timestamp_s > delay_update_time) {
      preintg_que.push_front(*it);
    } else if ((*it)->timestamp_s == delay_update_time) {
      preintg_que.push_front(*it);
      break;
    } else {
      auto it_1 = it - 1;
      auto dummy_imu =
          ImuHelper::imuLinearInterPolation(*it, *it_1, delay_update_time);
      preintg_que.push_front(dummy_imu);
      break;
    }
  }
  const auto cur_bias_gyro = state_manager_.getBiasGyroState();
  const auto cur_bias_accel = state_manager_.getBiasAccelState();
  const auto [deltaR, deltaP, deltaV] =
      ImuHelper::getPreintegThroughImuQue<std::deque<ImuMeasPtr>>(
          preintg_que, cur_bias_gyro, cur_bias_accel);
  double t_i_j =
      preintg_que.back()->timestamp_s - preintg_que.front()->timestamp_s;
  const auto R_j = state_manager_.getImuOriState().matrix();

  Eigen::Matrix<double, 9, 9> drpv_i_drpv_j;
  drpv_i_drpv_j.setZero();
  drpv_i_drpv_j.block<3, 3>(0, 0) = deltaR;
  const Eigen::Vector3d vec_temp_0 =
      deltaR.transpose() * (deltaV * t_i_j - deltaP);
  drpv_i_drpv_j.block<3, 3>(3, 0) = -R_j * skew(vec_temp_0);
  drpv_i_drpv_j.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  drpv_i_drpv_j.block<3, 3>(3, 6) = -t_i_j * Eigen::Matrix3d::Identity();
  const Eigen::Vector3d vec_temp_1 = deltaR.transpose() * deltaV;
  drpv_i_drpv_j.block<3, 3>(6, 0) = R_j * skew(vec_temp_1);
  drpv_i_drpv_j.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();

  delay_update_info.d_si_d_sj.topLeftCorner(9, 9) = drpv_i_drpv_j;
  delay_update_info.deltaR = deltaR;
  delay_update_info.deltaP = deltaP;
  delay_update_info.deltaV = deltaV;
  delay_update_info.deltaT = t_i_j;
  // std::cout << "getDelayUpdateInfo, drpv_i_drpv_j:" << std::endl
  //           << drpv_i_drpv_j << std::endl;
  // debug
  // delay_update_info.d_si_d_sj.setIdentity();
  // delay_update_info.deltaR.setIdentity();
  // delay_update_info.deltaP.setZero();
  // delay_update_info.deltaV.setZero();
  // delay_update_info.deltaT = 0;

  return delay_update_info;
}

void KfCoordinator::solveAndUpdate() {
  if (obv_res_.empty()) return;
  // process residual
  int res_dim = 0;
  for (auto res : obv_res_) {
    res_dim += res.size();
  }
  Eigen::VectorXd kf_deltaZ = Eigen::VectorXd::Zero(res_dim);
  int i = 0;
  for (auto& res : obv_res_) {
    kf_deltaZ.segment(i, res.size()) = res;
    i += res.size();
  }
  // std::cout << "kf_deltaZ: " << std::endl << kf_deltaZ << std::endl;

  // process jacobian
  int obv_dim = 0;
  for (auto& jacobian : obv_jacobian_) {
    obv_dim += jacobian.rows();
  }
  assert(res_dim == obv_dim);
  Eigen::MatrixXd kf_H = Eigen::MatrixXd::Zero(obv_dim, total_local_size);
  int row_idx = 0;
  for (auto& jacobian : obv_jacobian_) {
    kf_H.block(row_idx, 0, jacobian.rows(), total_local_size) = jacobian;
    row_idx += jacobian.rows();
  }
  // std::cout << "kf_H: " << std::endl << kf_H << std::endl;

  // set constant state's jacobian to zero
  std::vector<IdxSizePair> const_id_size =
      state_manager_.getConstStateIdxSize();
  for (auto& [start_idx, local_size] : const_id_size) {
    kf_H.middleCols(start_idx, local_size).setZero();
  }
  // kf update
  const Eigen::MatrixXd kf_K =
      state_cov_ * kf_H.transpose() *
      (kf_H * state_cov_ * kf_H.transpose() + obv_R).inverse();
  // std::cout << "kf_K: " << std::endl << kf_K << std::endl;
  const Eigen::VectorXd kf_deltaX = kf_K * kf_deltaZ;
  // std::cout << "kf_deltaX: " << std::endl << kf_deltaX << std::endl;
  const Eigen::MatrixXd kf_temp =
      Eigen::MatrixXd::Identity(total_local_size, total_local_size) -
      kf_K * kf_H;
  state_cov_ = kf_temp * state_cov_ * kf_temp.transpose() +
               kf_K * obv_R * kf_K.transpose();
  state_manager_.updateState(kf_deltaX);
  // std::cout << "state_cov_: " << std::endl << state_cov_ << std::endl;

  obv_res_.clear();
  obv_jacobian_.clear();
}

void KfCoordinator::updateWithOdomMeas(const OdomMeasPtr& odom_meas) {
  // get delay update jacobian
  DelayUpdateInfo du_info = getDelayUpdateInfo(odom_meas->timestamp_s);
  const Eigen::Vector3d gravity(0, 0, -kGravityMag);
  ROS_DEBUG("updateWithOdomMeas: t:%f, deltaT:%f.", odom_meas->timestamp_s,
            du_info.deltaT);
  // rot residual
  Sophus::SO3d obv_so3_w_b(odom_meas->q_w_b);
  Sophus::SO3d state_so3_w_i = state_manager_.getImuOriState();
  state_so3_w_i *= Sophus::SO3d(du_info.deltaR.transpose());
  const Eigen::Matrix3d R_w_i = state_so3_w_i.matrix();
  Sophus::SO3d state_so3_i_b = state_manager_.getExtOdomOriState();
  const Eigen::Matrix3d R_i_b = state_so3_i_b.matrix();
  Sophus::SO3d so3_w_b = state_so3_w_i * state_so3_i_b;
  Eigen::Vector3d residual_rot = (so3_w_b.inverse() * obv_so3_w_b).log();
  // rot jacobian
  std::vector<Eigen::MatrixXd> jacobians;
  std::vector<IdxSizePair> id_size_pairs;
  int rot_wi_idx = state_manager_.state_pool.at(getImuOriUid()).startIndex();
  int rot_wi_local_size =
      state_manager_.state_pool.at(getImuOriUid()).localSize();
  int rot_ib_idx =
      state_manager_.state_pool.at(getExtOdomOriUid()).startIndex();
  int rot_ib_local_size =
      state_manager_.state_pool.at(getExtOdomOriUid()).localSize();
  Eigen::Matrix3d d_zr_d_rwi = R_i_b.transpose();
  jacobians.push_back(d_zr_d_rwi);
  id_size_pairs.emplace_back(rot_wi_idx, rot_wi_local_size);
  Eigen::Matrix3d d_zr_d_rib = Eigen::Matrix3d::Identity();
  jacobians.push_back(d_zr_d_rib);
  id_size_pairs.emplace_back(rot_ib_idx, rot_ib_local_size);
  addResidualJacobian(residual_rot, jacobians, id_size_pairs,
                      du_info.d_si_d_sj);
  jacobians.clear();
  id_size_pairs.clear();

  // posi residual
  Eigen::Vector3d state_p_w_i = state_manager_.getImuTvecState();
  Eigen::Vector3d state_vel_w = state_manager_.getImuVelState();
  state_p_w_i = state_p_w_i - state_vel_w * du_info.deltaT +
                R_w_i * (du_info.deltaV * du_info.deltaT - du_info.deltaP) +
                0.5 * gravity * du_info.deltaT * du_info.deltaT;
  Eigen::Vector3d state_p_i_b = state_manager_.getExtOdomTvecState();
  Eigen::Vector3d p_w_b = R_w_i * state_p_i_b + state_p_w_i;
  Eigen::Vector3d residual_posi = odom_meas->p_w_b - p_w_b;
  // posi jacobian
  int posi_wi_idx = state_manager_.state_pool.at(getImuTvecUid()).startIndex();
  int posi_wi_local_size =
      state_manager_.state_pool.at(getImuTvecUid()).localSize();
  int posi_ib_idx =
      state_manager_.state_pool.at(getExtOdomTvecUid()).startIndex();
  int posi_ib_local_size =
      state_manager_.state_pool.at(getExtOdomTvecUid()).localSize();
  Eigen::Matrix3d d_zp_d_rwi = -R_w_i * skew(state_p_i_b);
  jacobians.push_back(d_zp_d_rwi);
  id_size_pairs.emplace_back(rot_wi_idx, rot_wi_local_size);
  Eigen::Matrix3d d_zp_d_pwi = Eigen::Matrix3d::Identity();
  jacobians.push_back(d_zp_d_pwi);
  id_size_pairs.emplace_back(posi_wi_idx, posi_wi_local_size);
  Eigen::Matrix3d d_zp_d_pib = R_w_i;
  jacobians.push_back(d_zp_d_pib);
  id_size_pairs.emplace_back(posi_ib_idx, posi_ib_local_size);
  addResidualJacobian(residual_posi, jacobians, id_size_pairs,
                      du_info.d_si_d_sj);
  jacobians.clear();
  id_size_pairs.clear();

  // vel residual
  state_vel_w = state_vel_w - R_w_i * du_info.deltaV - gravity * du_info.deltaT;
  Eigen::Vector3d residual_vel = odom_meas->vel_w - state_vel_w;
  // vel jacobian
  int vel_w_idx = state_manager_.state_pool.at(getImuVelUid()).startIndex();
  int vel_w_local_size =
      state_manager_.state_pool.at(getImuVelUid()).localSize();
  Eigen::Matrix3d d_zv_d_v = Eigen::Matrix3d::Identity();
  jacobians.push_back(d_zv_d_v);
  id_size_pairs.emplace_back(vel_w_idx, vel_w_local_size);
  addResidualJacobian(residual_vel, jacobians, id_size_pairs,
                      du_info.d_si_d_sj);
  jacobians.clear();
  id_size_pairs.clear();

  if (kf_config.use_motion_capture) {
    obv_R = Eigen::Matrix<double, 9, 9>::Identity();
    obv_R.block<3, 3>(0, 0) =
        kf_config.obv_mc_rot_cov.array().square().matrix().asDiagonal();
    obv_R.block<3, 3>(3, 3) =
        kf_config.obv_mc_posi_cov.array().square().matrix().asDiagonal();
    obv_R.block<3, 3>(6, 6) =
        kf_config.obv_mc_vel_cov.array().square().matrix().asDiagonal();
  } else if (kf_config.enable_merge_vins_bias) {
    // bias accel residual
    Eigen::Vector3d state_bias_accel = state_manager_.getBiasAccelState();
    Eigen::Vector3d residual_bias_accel =
        odom_meas->bias_accel - state_bias_accel;
    // bias accel jacobian
    int bias_accel_idx =
        state_manager_.state_pool.at(getImuBiasAccelUid()).startIndex();
    int bias_accel_local_size =
        state_manager_.state_pool.at(getImuBiasAccelUid()).localSize();
    Eigen::Matrix3d d_zba_d_ba = Eigen::Matrix3d::Identity();
    jacobians.push_back(d_zba_d_ba);
    id_size_pairs.emplace_back(bias_accel_idx, bias_accel_local_size);
    addResidualJacobian(residual_bias_accel, jacobians, id_size_pairs,
                        du_info.d_si_d_sj);

    // set obv_R(fixed)
    obv_R = Eigen::Matrix<double, 12, 12>::Identity();
    obv_R.block<3, 3>(0, 0) =
        kf_config.obv_odom_rot_cov.array().square().matrix().asDiagonal();
    obv_R.block<3, 3>(3, 3) =
        kf_config.obv_odom_posi_cov.array().square().matrix().asDiagonal();
    obv_R.block<3, 3>(6, 6) =
        kf_config.obv_odom_vel_cov.array().square().matrix().asDiagonal();
    obv_R.block<3, 3>(9, 9) =
        kf_config.obv_vins_bias_cov.array().square().matrix().asDiagonal();
  } else {
    obv_R = Eigen::Matrix<double, 9, 9>::Identity();
    obv_R.block<3, 3>(0, 0) =
        kf_config.obv_odom_rot_cov.array().square().matrix().asDiagonal();
    obv_R.block<3, 3>(3, 3) =
        kf_config.obv_odom_posi_cov.array().square().matrix().asDiagonal();
    obv_R.block<3, 3>(6, 6) =
        kf_config.obv_odom_vel_cov.array().square().matrix().asDiagonal();
  }
  // solve
  solveAndUpdate();
}

void KfCoordinator::updateWithZUPT(const ImuMeasPtr& imu_meas) {
  std::vector<Eigen::MatrixXd> jacobians;
  std::vector<IdxSizePair> id_size_pairs;
  // gyro residual
  Eigen::Vector3d state_bias_gyro = state_manager_.getBiasGyroState();
  Eigen::Vector3d residual_gyro = -(imu_meas->gyr_rad_s - state_bias_gyro);
  // gyro jacobian
  int bias_gyro_idx =
      state_manager_.state_pool.at(getImuBiasGyroUid()).startIndex();
  int bias_gyro_local_size =
      state_manager_.state_pool.at(getImuBiasGyroUid()).localSize();
  Eigen::Matrix3d d_zg_d_bg = -Eigen::Matrix3d::Identity();
  jacobians.push_back(d_zg_d_bg);
  id_size_pairs.emplace_back(bias_gyro_idx, bias_gyro_local_size);
  addResidualJacobian(residual_gyro, jacobians, id_size_pairs,
                      DUMatType::Identity());
  jacobians.clear();
  id_size_pairs.clear();

  // accel residual
  Eigen::Matrix3d R_w_i = state_manager_.getImuOriState().matrix();
  Eigen::Vector3d state_bias_accel = state_manager_.getBiasAccelState();
  Eigen::Vector3d residual_accel =
      -(R_w_i * (imu_meas->acc_m_s2 - state_bias_accel) +
        Eigen::Vector3d(0, 0, -kGravityMag));
  // accel jacobian
  int rot_idx = state_manager_.state_pool.at(getImuOriUid()).startIndex();
  int rot_local_size = state_manager_.state_pool.at(getImuOriUid()).localSize();
  int bias_accel_idx =
      state_manager_.state_pool.at(getImuBiasAccelUid()).startIndex();
  int bias_accel_local_size =
      state_manager_.state_pool.at(getImuBiasAccelUid()).localSize();
  Eigen::Vector3d acc_temp = imu_meas->acc_m_s2 - state_bias_accel;
  Eigen::Matrix3d d_za_d_rwi = -R_w_i * skew(acc_temp);
  jacobians.push_back(d_za_d_rwi);
  id_size_pairs.emplace_back(rot_idx, rot_local_size);
  Eigen::Matrix3d d_za_d_ba = -R_w_i;
  jacobians.push_back(d_za_d_ba);
  id_size_pairs.emplace_back(bias_accel_idx, bias_accel_local_size);
  addResidualJacobian(residual_accel, jacobians, id_size_pairs,
                      DUMatType::Identity());
  jacobians.clear();
  id_size_pairs.clear();

  // vel residual
  Eigen::Vector3d vel_w = state_manager_.getImuVelState();
  Eigen::Vector3d residual_v = -vel_w;
  // vel jacobian
  int vel_idx = state_manager_.state_pool.at(getImuVelUid()).startIndex();
  int vel_local_size = state_manager_.state_pool.at(getImuVelUid()).localSize();
  jacobians.push_back(Eigen::Matrix3d::Identity());
  id_size_pairs.emplace_back(vel_idx, vel_local_size);
  addResidualJacobian(residual_v, jacobians, id_size_pairs,
                      DUMatType::Identity());

  // set obv_R
  obv_R = Eigen::Matrix<double, 9, 9>::Identity();
  obv_R.block<3, 3>(0, 0) =
      kf_config.obv_zupt_gyro_cov.array().square().matrix().asDiagonal();
  obv_R.block<3, 3>(3, 3) =
      kf_config.obv_zupt_accel_cov.array().square().matrix().asDiagonal();
  obv_R.block<3, 3>(6, 6) =
      kf_config.obv_zupt_vel_cov.array().square().matrix().asDiagonal();

  solveAndUpdate();
}