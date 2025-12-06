#include "kf_nav/kf_interface.h"

#include "std_msgs/Bool.h"

void KfInterface::receiveImuTopic(const sensor_msgs::ImuConstPtr& imu_msg) {
  MeasureBasePtr meas_ptr = ImuMeas::createPtr(imu_msg);
  meas_que_.push(meas_ptr);
  cv_msg_.notify_all();
}

void KfInterface::receiveOdomTopic(const nav_msgs::OdometryConstPtr& odom_msg) {
  MeasureBasePtr meas_ptr = OdomMeas::createPtr(odom_msg);
  meas_que_.push(meas_ptr);
  cv_msg_.notify_all();
}

void KfInterface::processMeasurements() {
  while (ros::ok() && !ros::isShuttingDown()) {
    std::unique_lock<std::mutex> lock(mtx_msg_);
    if (cv_msg_.wait_for(lock, std::chrono::milliseconds(100),
                         [&]() { return !meas_que_.empty(); })) {
      std::vector<MeasureBasePtr> meas_batch = meas_que_.pop_all();
      for (auto& meas : meas_batch) {
        if (kf_coordiantor_.isInit() == false) {
          if (state_initializer_.initState(meas) == true) {
            kf_coordiantor_.init(state_initializer_.getInitialState());
            state_initializer_.clearBuffer();
          }
        } else {
          switch (meas->type) {
            case MeasureType::kImu: {
              auto imu_meas = std::static_pointer_cast<ImuMeas>(meas);
              kf_coordiantor_.processImuMeas(imu_meas);
              static_check_.inputImu(imu_meas);
              if (static_check_.isStatic()) {
                kf_coordiantor_.updateWithZUPT(imu_meas);
                ROS_WARN_THROTTLE(1, "Static Checked!");
              }
              publishStatic(static_check_.isStatic());
              break;
            }
            case MeasureType::kOdometry: {
              if (static_check_.isStatic()) break;
              auto odom_meas = std::static_pointer_cast<OdomMeas>(meas);
              kf_coordiantor_.updateWithOdomMeas(odom_meas);
              publishObvOdom(odom_meas);
              break;
            }
            default:
              break;
          }
          publishImuOdom(meas->timestamp_s);
          publishExt(meas->timestamp_s);
        }
      }
    } else {
      if (!ros::ok()) {
        break;
      }
    }
  }
}

void KfInterface::init(ros::NodeHandle& nh) {
  pub_imu_odom_ = nh.advertise<nav_msgs::Odometry>("kf_imu_odom", 1000);
  pub_imu_path_ = nh.advertise<nav_msgs::Path>("kf_imu_path", 1000);
  pub_obv_odom_ = nh.advertise<nav_msgs::Odometry>("kf_obv_odom", 1000);
  pub_obv_path_ = nh.advertise<nav_msgs::Path>("kf_obv_path", 1000);
  pub_static_ = nh.advertise<std_msgs::Bool>("kf_static_check", 10);
  pub_ext_ = nh.advertise<geometry_msgs::PoseStamped>("kf_ext", 10);
  sub_imu_ = nh.subscribe(config_.imu_topic, 1000,
                          &KfInterface::receiveImuTopic, this);
  sub_odom_ = nh.subscribe(config_.odom_topic, 100,
                           &KfInterface::receiveOdomTopic, this);
}

void KfInterface::publishImuOdom(double t) {
  Sophus::SO3d state_ori =
      kf_coordiantor_.getStateManagerRef().getImuOriState();
  Eigen::Quaterniond state_oir_q = state_ori.unit_quaternion();
  Eigen::Vector3d state_posi =
      kf_coordiantor_.getStateManagerRef().getImuTvecState();
  Eigen::Vector3d state_vel =
      kf_coordiantor_.getStateManagerRef().getImuVelState();
  Eigen::Vector3d bias_accel =
      kf_coordiantor_.getStateManagerRef().getBiasAccelState();

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time(t);
  odom_msg.header.frame_id = "world";
  odom_msg.pose.pose.orientation.x = state_oir_q.x();
  odom_msg.pose.pose.orientation.y = state_oir_q.y();
  odom_msg.pose.pose.orientation.z = state_oir_q.z();
  odom_msg.pose.pose.orientation.w = state_oir_q.w();
  odom_msg.pose.pose.position.x = state_posi.x();
  odom_msg.pose.pose.position.y = state_posi.y();
  odom_msg.pose.pose.position.z = state_posi.z();
  odom_msg.twist.twist.linear.x = state_vel.x();
  odom_msg.twist.twist.linear.y = state_vel.y();
  odom_msg.twist.twist.linear.z = state_vel.z();
  odom_msg.twist.twist.angular.x = bias_accel.x();
  odom_msg.twist.twist.angular.y = bias_accel.y();
  odom_msg.twist.twist.angular.z = bias_accel.z();
  pub_imu_odom_.publish(odom_msg);

  geometry_msgs::PoseStamped imu_pose_stamped;
  imu_pose_stamped.header.frame_id = "world";
  imu_pose_stamped.header.stamp = ros::Time(t);
  imu_pose_stamped.pose = odom_msg.pose.pose;
  imu_path_.header.stamp = ros::Time(t);
  imu_path_.header.frame_id = "world";
  imu_path_.poses.push_back(imu_pose_stamped);
  pub_imu_path_.publish(imu_path_);

  Eigen::Vector3d euler_angles =
      state_oir_q.toRotationMatrix().eulerAngles(2, 1, 0);
  euler_angles = euler_angles / (2 * M_PI) * 180.0;
  ROS_WARN_THROTTLE(0.5,
                    "time: %f, t: %f %f %f , yaw: %f , pitch: %f , roll: %f ",
                    t, state_posi.x(), state_posi.y(), state_posi.z(),
                    euler_angles.x(), euler_angles.y(), euler_angles.z());
}

void KfInterface::publishObvOdom(const OdomMeasPtr& odom_meas) {
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time(odom_meas->timestamp_s);
  odom_msg.header.frame_id = "world";
  odom_msg.pose.pose.orientation.x = odom_meas->q_w_b.x();
  odom_msg.pose.pose.orientation.y = odom_meas->q_w_b.y();
  odom_msg.pose.pose.orientation.z = odom_meas->q_w_b.z();
  odom_msg.pose.pose.orientation.w = odom_meas->q_w_b.w();
  odom_msg.pose.pose.position.x = odom_meas->p_w_b.x();
  odom_msg.pose.pose.position.y = odom_meas->p_w_b.y();
  odom_msg.pose.pose.position.z = odom_meas->p_w_b.z();
  odom_msg.twist.twist.linear.x = odom_meas->vel_w.x();
  odom_msg.twist.twist.linear.y = odom_meas->vel_w.y();
  odom_msg.twist.twist.linear.z = odom_meas->vel_w.z();
  pub_obv_odom_.publish(odom_msg);

  geometry_msgs::PoseStamped obv_pose_stamped;
  obv_pose_stamped.header.frame_id = "world";
  obv_pose_stamped.header.stamp = ros::Time(odom_meas->timestamp_s);
  obv_pose_stamped.pose = odom_msg.pose.pose;
  obv_path_.header.stamp = ros::Time(odom_meas->timestamp_s);
  obv_path_.header.frame_id = "world";
  obv_path_.poses.push_back(obv_pose_stamped);
  pub_obv_path_.publish(obv_path_);
}

void KfInterface::publishStatic(bool is_static) {
  std_msgs::Bool msg;
  msg.data = is_static;
  pub_static_.publish(msg);
}

void KfInterface::publishExt(double t) {
  Sophus::SO3d state_ext_ori =
      kf_coordiantor_.getStateManagerRef().getExtOdomOriState();
  Eigen::Vector3d state_ext_tvec =
      kf_coordiantor_.getStateManagerRef().getExtOdomTvecState();

  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time(t);
  msg.header.frame_id = "world";
  msg.pose.orientation.x = state_ext_ori.unit_quaternion().x();
  msg.pose.orientation.y = state_ext_ori.unit_quaternion().y();
  msg.pose.orientation.z = state_ext_ori.unit_quaternion().z();
  msg.pose.orientation.w = state_ext_ori.unit_quaternion().w();
  msg.pose.position.x = state_ext_tvec.x();
  msg.pose.position.y = state_ext_tvec.y();
  msg.pose.position.z = state_ext_tvec.z();
  pub_ext_.publish(msg);
}