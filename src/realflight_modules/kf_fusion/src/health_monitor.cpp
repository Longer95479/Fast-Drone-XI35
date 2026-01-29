#include "kf_nav/health_monitor.h"
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
    if (is_vins_invalid_) {
        mtx_val.lock();
        if (health_val_ > 0) {
            health_val_ = -0.1;
        }
        mtx_val.unlock();
        return;
    }
    mtx_val.lock();
    health_val_ += vis_update_increment;
    health_val_ = health_val_ > maxHealthValue ? maxHealthValue : health_val_;
    mtx_val.unlock();
}

void HealthMonitor::updateFromMcUpt(double ts) {
    mtx_val.lock();
    health_val_ += mc_update_increment;
    health_val_ = health_val_ > maxHealthValue ? maxHealthValue : health_val_;
    mtx_val.unlock();
}

void HealthMonitor::updateFromZupt(double ts) {
    mtx_val.lock();
    health_val_ += zupt_update_increment;
    health_val_ = health_val_ > maxHealthValue ? maxHealthValue : health_val_;
    mtx_val.unlock();
}

bool HealthMonitor::isKfHealthy() {
    if (health_val_ < 0) {
        ROS_WARN("HealthMonitor: health value is 0!");
        return false;
    }

    double acc_bias_norm = kf_coordiantor_ptr_->getStateManagerRef().getBiasAccelState().norm();
    if (acc_bias_norm > 2.5) {
        ROS_WARN("HealthMonitor: big IMU acc bias estimation %f.", acc_bias_norm);
        return false;
    }

    double gyro_bias_norm = kf_coordiantor_ptr_->getStateManagerRef().getBiasGyroState().norm();
    if (gyro_bias_norm > 1.0) {
        ROS_WARN("HealthMonitor: big IMU gyro bias estimation %f.", gyro_bias_norm);
        return false;
    }

    double posi_norm = kf_coordiantor_ptr_->getStateManagerRef().getImuTvecState().norm();
    if (posi_norm > maxTranslation) {
        ROS_WARN("HealthMonitor: big position estimation %f.", posi_norm);
        return false;
    }

    double v_norm = kf_coordiantor_ptr_->getStateManagerRef().getImuVelState().norm();
    if (v_norm > maxVelocity) {
        ROS_WARN("HealthMonitor: big velocity estimation %f.", v_norm);
        return false;
    }

    return true;
}

void HealthMonitor::CheckAndPublish(ros::Publisher& pub_fail) {
    if (!isKfHealthy()) {
        std_msgs::Bool msg;
        msg.data = true;
        pub_fail.publish(msg);
    }
}