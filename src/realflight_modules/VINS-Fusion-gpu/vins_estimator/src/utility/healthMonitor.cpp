#include "../estimator/estimator.h"
#include "std_msgs/Bool.h"

void HealthMonitor::updateFromImuProp(double ts) {
    if (ts_last_imu_prop_ < 0) {
        ts_last_imu_prop_ = ts;
        return;
    }
    double prop_dura = ts - ts_last_imu_prop_;
    if (prop_dura < 0) {
        ROS_WARN("prop_dura is negative.");
        return;
    }
    mtx_val.lock();
    health_val_ -= prop_dura;
    mtx_val.unlock();
    ts_last_imu_prop_ = ts;
    return;
}

void HealthMonitor::updateFromVisUpt(double ts) {
    int cur_feat_num = feat_manager_ptr_->getFeatureCount();
    double increment = 0;
    if (cur_feat_num > visFeatCntMax) {
        increment = 0.2;
    } else {
        increment = static_cast<double>(cur_feat_num) / visFeatCntMax * 0.2;
    }
    ROS_DEBUG(
        "HealthMonitor: update from visual, cur_feat_num is %d, increment is %f.", cur_feat_num,
        increment);
    mtx_val.lock();
    health_val_ += increment;
    health_val_ = health_val_ > maxHealthValue ? maxHealthValue : health_val_;
    mtx_val.unlock();
    ts_last_upt_ = ts;
}

bool HealthMonitor::isVINSHealthy() {
    if (health_val_ < 0) {
        ROS_WARN("HealthMonitor: health value is 0!");
        return false;
    }
    if (estimator_ptr_->Bas[WINDOW_SIZE].norm() > 2.5) {
        ROS_WARN(
            "HealthMonitor: big IMU acc bias estimation %f.",
            estimator_ptr_->Bas[WINDOW_SIZE].norm());
        return false;
    }
    if (estimator_ptr_->Bgs[WINDOW_SIZE].norm() > 1.0) {
        ROS_WARN(
            "HealthMonitor: big IMU gyro bias estimation %f.",
            estimator_ptr_->Bgs[WINDOW_SIZE].norm());
        return false;
    }
    if (estimator_ptr_->Ps[WINDOW_SIZE].norm() > maxTranslation) {
        ROS_WARN(
            "HealthMonitor: big position estimation %f.", estimator_ptr_->Ps[WINDOW_SIZE].norm());
        return false;
    }
    if (estimator_ptr_->Vs[WINDOW_SIZE].norm() > maxVelocity) {
        ROS_WARN(
            "HealthMonitor: big velocity estimation %f.", estimator_ptr_->Vs[WINDOW_SIZE].norm());
        return false;
    }
    return true;
}

void HealthMonitor::CheckAndPublish(ros::Publisher &pub_fail) {
    if (!isVINSHealthy()) {
        std_msgs::Bool msg;
        msg.data = true;
        pub_fail.publish(msg);
    }
}