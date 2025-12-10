#include "kf_nav/ext_calib.h"

template <typename T>
Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1>& vec) {
  return (Eigen::Matrix<T, 3, 3>() << T(0), -vec(2), vec(1), vec(2), T(0),
          -vec(0), -vec(1), vec(0), T(0))
      .finished();
}

void ExtCalib::receive_vins_msg(const nav_msgs::OdometryPtr& msg) {
  Eigen::Quaterniond q;
  Eigen::Vector3d p;
  q.w() = msg->pose.pose.orientation.w;
  q.x() = msg->pose.pose.orientation.x;
  q.y() = msg->pose.pose.orientation.y;
  q.z() = msg->pose.pose.orientation.z;
  p.x() = msg->pose.pose.position.x;
  p.y() = msg->pose.pose.position.y;
  p.z() = msg->pose.pose.position.z;

  auto vins_pose_ptr =
      OdomPose::create(msg->header.stamp.toSec(), q.normalized(), p);
  vins_msg_queue_.push(vins_pose_ptr);
}

void ExtCalib::receive_mc_msg(const nav_msgs::OdometryPtr& msg) {
  Eigen::Quaterniond q;
  Eigen::Vector3d p;
  q.w() = msg->pose.pose.orientation.w;
  q.x() = msg->pose.pose.orientation.x;
  q.y() = msg->pose.pose.orientation.y;
  q.z() = msg->pose.pose.orientation.z;
  p.x() = msg->pose.pose.position.x;
  p.y() = msg->pose.pose.position.y;
  p.z() = msg->pose.pose.position.z;

  auto mc_pose_ptr =
      OdomPose::create(msg->header.stamp.toSec(), q.normalized(), p);
  mc_msg_queue_.push(mc_pose_ptr);
}

bool ExtCalib::judgeMotion(const OdomPosePtr& last_pose,
                           const OdomPosePtr& cur_pose) {
  double angle_dist = last_pose->q_w_i.angularDistance(cur_pose->q_w_i);
  angle_dist = angle_dist / M_PI * 180.0;
  if (angle_dist < 5.0) return false;
  double posi_dist = (cur_pose->p_w_i - last_pose->p_w_i).norm();
  if (posi_dist < 0.1) return false;
  return true;
}

std::optional<OdomPosePtr> ExtCalib::interpolAndPopMcPose(double t) {
  if (mc_msg_queue_.empty() || t < mc_msg_queue_.front().value()->timestamp ||
      t > mc_msg_queue_.back().value()->timestamp)
    return std::nullopt;
  OdomPosePtr left_pose_ptr;
  while (!mc_msg_queue_.empty()) {
    if (mc_msg_queue_.front().value()->timestamp < t) {
      left_pose_ptr = mc_msg_queue_.front().value();
      mc_msg_queue_.pop();
    } else if (mc_msg_queue_.front().value()->timestamp == t) {
      return mc_msg_queue_.front().value();
    } else {
      break;
    }
  }
  if (mc_msg_queue_.empty() || left_pose_ptr == nullptr) return std::nullopt;
  auto right_pose_ptr = mc_msg_queue_.front().value();
  double slerp_t = (t - left_pose_ptr->timestamp) /
                   (right_pose_ptr->timestamp - left_pose_ptr->timestamp);
  Eigen::Quaterniond slerp_q =
      left_pose_ptr->q_w_i.slerp(slerp_t, right_pose_ptr->q_w_i).normalized();
  double coffe_right = slerp_t;
  double coffe_left = 1 - slerp_t;
  Eigen::Vector3d linear_p =
      coffe_left * left_pose_ptr->p_w_i + coffe_right * right_pose_ptr->p_w_i;
  return OdomPose::create(t, slerp_q, linear_p);
}

bool ExtCalib::solveExtPose() {
  if (vins_solve_pose_.size() < 50) return false;
  // calib ext_ori
  if (config_.calib_ori) {
    Eigen::MatrixXd A((vins_solve_pose_.size() - 1) * 4, 4);
    for (int i = 1; i < vins_solve_pose_.size(); i++) {
      Eigen::Quaterniond delta_q_i =
          vins_solve_pose_[i - 1]->q_w_i.inverse() * vins_solve_pose_[i]->q_w_i;
      Eigen::Quaterniond delta_q_m =
          mc_solve_pose_[i - 1]->q_w_i.inverse() * mc_solve_pose_[i]->q_w_i;
      Eigen::Quaterniond delta_q_m_temp =
          ext_q_ib_.inverse() * delta_q_i * ext_q_ib_;

      double angle_dist =
          delta_q_m.angularDistance(delta_q_m_temp) / M_PI * 180.0;
      double huber = angle_dist > 5.0 ? 5.0 / angle_dist : 1.0;

      Eigen::Matrix4d L, R;

      double w = delta_q_i.w();
      Eigen::Vector3d v = delta_q_i.vec();
      L.block<3, 3>(0, 0) = w * Eigen::Matrix3d::Identity() + skew(v);
      L.block<3, 1>(0, 3) = v;
      L.block<1, 3>(3, 0) = -v.transpose();
      L(3, 3) = w;

      w = delta_q_m.w();
      v = delta_q_m.vec();
      R.block<3, 3>(0, 0) = w * Eigen::Matrix3d::Identity() - skew(v);
      R.block<3, 1>(0, 3) = v;
      R.block<1, 3>(3, 0) = -v.transpose();
      R(3, 3) = w;

      A.block<4, 4>((i - 1) * 4, 0) = huber * (L - R);
    }
    Eigen::JacobiSVD svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector4d singular_vec = svd.singularValues();
    if (singular_vec(2) > 0.25) {
      Eigen::Vector4d x = svd.matrixV().col(3);
      ext_q_ib_ = Eigen::Quaterniond(x).normalized();
      ROS_WARN("Solve external rotation successfully!");
      std::cout << "External rotation calib result is:" << std::endl
                << ext_q_ib_.toRotationMatrix() << std::endl;
    } else {
      ROS_WARN("Solve external rotation failed!");
      std::cout << "SVD singular value is: " << singular_vec << std::endl;
      return false;
    }
  }
  // calib ext_posi
  Eigen::MatrixXd A((vins_solve_pose_.size() - 1) * 3, 3);
  Eigen::VectorXd b((vins_solve_pose_.size() - 1) * 3);
  for (int i = 1; i < vins_solve_pose_.size(); i++) {
    Eigen::Quaterniond delta_q_i =
        vins_solve_pose_[i - 1]->q_w_i.inverse() * vins_solve_pose_[i]->q_w_i;
    Eigen::Matrix3d delta_R_i = delta_q_i.toRotationMatrix();
    Eigen::Vector3d delta_t_i =
        vins_solve_pose_[i - 1]->q_w_i.toRotationMatrix().transpose() *
        (vins_solve_pose_[i]->p_w_i - vins_solve_pose_[i - 1]->p_w_i);

    Eigen::Vector3d delta_t_m =
        mc_solve_pose_[i - 1]->q_w_i.toRotationMatrix().transpose() *
        (mc_solve_pose_[i]->p_w_i - mc_solve_pose_[i - 1]->p_w_i);

    A.block<3, 3>((i - 1) * 3, 0) = delta_R_i - Eigen::Matrix3d::Identity();
    b.segment<3>((i - 1) * 3) =
        ext_q_ib_.toRotationMatrix() * delta_t_m - delta_t_i;
  }
  ext_p_ib_ = A.colPivHouseholderQr().solve(b);
  if (ext_p_ib_.hasNaN() || !ext_p_ib_.allFinite()) {
    ROS_WARN("Solve external position failed!");
    return false;
  } else {
    ROS_WARN("Solve external position successfully!.");
    std::cout << "External position calib result is:" << std::endl
              << ext_p_ib_ << std::endl;
    return true;
  }
}

void ExtCalib::init(ros::NodeHandle& nh) {
  config_.loadConfig();
  sub_vins_odom_ =
      nh.subscribe(config_.vins_topic, 1000, &ExtCalib::receive_vins_msg, this);
  sub_mc_odom_ =
      nh.subscribe(config_.mc_topic, 1000, &ExtCalib::receive_mc_msg, this);
  ext_q_ib_ = config_.ext_q_init;
  thrd_hdl_ = std::thread(&ExtCalib::processData, this);
}

void ExtCalib::processData() {
  while (ros::ok() && !ros::isShuttingDown()) {
    switch (cur_state_) {
      case ExtCalib::State::ready: {
        if (vins_msg_queue_.empty()) break;
        auto first_vins_pose = vins_msg_queue_.front().value();
        if (auto intpol_mc_pose_opt =
                interpolAndPopMcPose(first_vins_pose->timestamp);
            intpol_mc_pose_opt.has_value()) {
          vins_solve_pose_.push_back(first_vins_pose);
          mc_solve_pose_.push_back(intpol_mc_pose_opt.value());
          cur_state_ = ExtCalib::State::dataCollection;
          ROS_WARN("Calibration start, please move the drone fully.");
        }
        vins_msg_queue_.pop();
        break;
      }
      case ExtCalib::State::dataCollection: {
        if (vins_msg_queue_.empty()) {
          sequal_invalid_pose_cnt_++;
          break;
        }
        auto last_vins_solve_pose = vins_solve_pose_.back();
        auto cur_vins_msg_pose = vins_msg_queue_.front().value();
        if (judgeMotion(last_vins_solve_pose, cur_vins_msg_pose)) {
          auto intpol_mc_pose_opt =
              interpolAndPopMcPose(cur_vins_msg_pose->timestamp);
          if (intpol_mc_pose_opt.has_value()) {
            sequal_invalid_pose_cnt_ = 0;
            vins_solve_pose_.push_back(cur_vins_msg_pose);
            mc_solve_pose_.push_back(intpol_mc_pose_opt.value());
            ROS_INFO("Valid pose detected and added, vector total size is %ld.",
                     vins_solve_pose_.size());
          }
        } else {
          if (vins_solve_pose_.size() > 50) {
            sequal_invalid_pose_cnt_++;
            if (sequal_invalid_pose_cnt_ >= 1000) {
              cur_state_ = ExtCalib::State::solving;
              ROS_WARN("Collecting data finished, total size is %ld.",
                       vins_solve_pose_.size());
            }
          }
        }
        vins_msg_queue_.pop();
        break;
      }
      case ExtCalib::State::solving: {
        ROS_WARN("Start solve external pose.");
        assert(vins_solve_pose_.size() == mc_solve_pose_.size());
        if (solveExtPose()) {
          ROS_WARN("Calibration successful, program terminated.");
          cur_state_ = ExtCalib::State::termination;
        } else {
          ROS_WARN("Calibration failed, will restart.");
          cur_state_ = ExtCalib::State::ready;
        }
        vins_solve_pose_.clear();
        mc_solve_pose_.clear();
        break;
      }
      case ExtCalib::State::termination: {
        break;
      }
      default:
        break;
    }

    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}
