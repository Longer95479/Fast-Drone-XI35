#include "kf_nav/state_initializer.h"

#include "kf_nav/imu_helper.h"

bool StateInitializer::initState(const MeasureBasePtr& meas) {
  switch (meas->type) {
    case MeasureType::kImu: {
      imu_que_.push_back(std::static_pointer_cast<ImuMeas>(meas));
      if (getImuQueueDura() > static_check_dura_s_) {
        if (!is_bias_init_) {  // only init bias once
          is_bias_init_ = initGyroBias();
        }
        while (getImuQueueDura() > static_check_dura_s_) {
          imu_que_.pop_front();
        }
      }
      break;
    }
    case MeasureType::kOdometry: {
      // only keep latest odom
      if (!odom_que_.empty() &&
          meas->timestamp_s > odom_que_.back()->timestamp_s) {
        while (!odom_que_.empty()) {
          odom_que_.pop_front();
        }
      }
      odom_que_.push_back(std::static_pointer_cast<OdomMeas>(meas));
      if (is_bias_init_) {
        is_state_init_ = initByOdom();
      }
      break;
    }
    default:
      break;
  }

  return is_state_init_;
}

double StateInitializer::getImuQueueDura() {
  if (imu_que_.size() < 3) return 0;
  auto oldest_time = imu_que_.front()->timestamp_s;
  auto latest_time = imu_que_.back()->timestamp_s;
  return std::fabs(latest_time - oldest_time);
}

bool StateInitializer::initGyroBias() {
  if (imu_que_.size() < 10) return false;
  Eigen::Vector3d gyro_avg{0, 0, 0};
  Eigen::Vector3d accel_avg{0, 0, 0};
  for (auto& imu_ptr : imu_que_) {
    gyro_avg += imu_ptr->gyr_rad_s;
    accel_avg += imu_ptr->acc_m_s2;
  }
  gyro_avg *= (1. / imu_que_.size());
  accel_avg *= (1. / imu_que_.size());

  double gyro_var = 0.;
  double accel_var = 0.;
  for (auto& imu_ptr : imu_que_) {
    gyro_var +=
        (imu_ptr->gyr_rad_s - gyro_avg).dot(imu_ptr->gyr_rad_s - gyro_avg);
    accel_var +=
        (imu_ptr->acc_m_s2 - accel_avg).dot(imu_ptr->acc_m_s2 - accel_avg);
  }
  gyro_var /= (-1. + imu_que_.size());
  accel_var /= (-1. + imu_que_.size());

  double gyro_avg_norm = gyro_avg.norm();
  double accel_avg_norm = std::fabs(accel_avg.norm() - kGravityMag);
  if (gyro_avg_norm < thresh_gyro_avg_ && accel_avg_norm < thresh_accel_avg_ &&
      gyro_var < thresh_gyro_var_ && accel_var < thresh_accel_var_) {
    state_.bias_gyro = gyro_avg;
    ROS_WARN(
        "StateInitializer: bias initializing successfully! bias_gyro:[%f, %f, "
        "%f].",
        state_.bias_gyro[0], state_.bias_gyro[1], state_.bias_gyro[2]);
    ROS_INFO(
        "StateInitializer: gyro_avg_norm: %f, accel_avg_norm: %f, gyro_var: "
        "%f, accel_var: %f, the que size is %ld",
        gyro_avg_norm, accel_avg_norm, gyro_var, accel_var, imu_que_.size());
    return true;
  }
  return false;
}

bool StateInitializer::initByOdom() {
  if (!is_bias_init_ || odom_que_.empty()) return false;
  const auto& odom_latest = odom_que_.back();
  auto odom_time = odom_latest->timestamp_s;
  if (odom_time > imu_que_.back()->timestamp_s + 0.02 ||
      odom_time < imu_que_.front()->timestamp_s)
    return false;
  // odom's time >= latest imu time
  if (odom_time >= imu_que_.back()->timestamp_s) {
    state_.q_w_i = odom_latest->q_w_b * (state_.q_i_b.normalized().inverse());
    state_.q_w_i.normalize();
    state_.p_w_i = odom_latest->p_w_b - state_.q_w_i * state_.p_i_b;
    state_.vel_w = odom_latest->vel_w;
    state_.timestamp_s = imu_que_.back()->timestamp_s;
    state_.gyro_init = imu_que_.back()->gyr_rad_s;
    state_.accel_init = imu_que_.back()->acc_m_s2;
    return true;
  }
  // odom's time < latest imu time
  std::deque<ImuMeasPtr> preintg_que;
  for (auto it = imu_que_.rbegin(); it != imu_que_.rend(); it++) {
    if ((*it)->timestamp_s > odom_time) {
      preintg_que.push_front(*it);
    } else if ((*it)->timestamp_s == odom_time) {
      preintg_que.push_front(*it);
      break;
    } else {
      auto it_1 = it - 1;
      auto dummy_imu = ImuHelper::imuLinearInterPolation(*it, *it_1, odom_time);
      preintg_que.push_front(dummy_imu);
      break;
    }
  }
  ROS_INFO("StateInitializer: preintg_que time length is [%f, %f].",
           preintg_que.front()->timestamp_s, preintg_que.back()->timestamp_s);
  const auto [deltaR, deltaP, deltaV] =
      ImuHelper::getPreintegThroughImuQue<std::deque<ImuMeasPtr>>(
          preintg_que, state_.bias_gyro, state_.bias_accel);
  const Eigen::Vector3d g = {0., 0., -kGravityMag};
  double deltaT =
      preintg_que.back()->timestamp_s - preintg_que.front()->timestamp_s;
  Eigen::Matrix3d R_odom_w_i =
      odom_latest->q_w_b.matrix() * state_.q_i_b.matrix().transpose();
  Eigen::Vector3d p_odom_w_i = odom_latest->p_w_b - R_odom_w_i * state_.p_i_b;
  Eigen::Matrix3d R_latest_w_i = R_odom_w_i * deltaR;
  Eigen::Vector3d p_latest_w_i = p_odom_w_i + odom_latest->vel_w * deltaT +
                                 R_odom_w_i * deltaP +
                                 0.5 * g * deltaT * deltaT;
  Eigen::Vector3d v_latest_w =
      odom_latest->vel_w + R_odom_w_i * deltaV + g * deltaT;
  state_.q_w_i = Eigen::Quaterniond(R_latest_w_i);
  state_.p_w_i = p_latest_w_i;
  state_.vel_w = v_latest_w;
  state_.timestamp_s = imu_que_.back()->timestamp_s;
  state_.gyro_init = imu_que_.back()->gyr_rad_s;
  state_.accel_init = imu_que_.back()->acc_m_s2;
  ROS_WARN("StateInitializer: odom initializing successfully!");
  return true;
}

void StateInitializer::clearBuffer() {
  imu_que_.clear();
  odom_que_.clear();
}