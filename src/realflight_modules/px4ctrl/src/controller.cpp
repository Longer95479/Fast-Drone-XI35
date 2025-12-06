#include "controller.h"
#include "PX4CtrlFSM.h"
#include "input.h"
#include <type_traits>

using namespace std;

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q) {
    double yaw = atan2(
        2 * (q.x() * q.y() + q.w() * q.z()),
        q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return yaw;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param) { resetThrustMapping(); }

/*
  compute u.thrust and u.q, controller gains and other parameters are in param_
*/
quadrotor_msgs::Px4ctrlDebug LinearControl::calculateControl(
    const Desired_State_t &des, const Odom_Data_t &odom, const Imu_Data_t &imu,
    Controller_Output_t &u) {
    /* WRITE YOUR CODE HERE */
    // compute disired acceleration
    Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
    Eigen::Vector3d Kp, Kd, Ki;
    Eigen::Vector3d err_v, err_p;
    Eigen::Vector3d des_vel_fb, des_acc_fb;
    Eigen::Vector3d vel_inte_part, vel_diff_part;
    static Eigen::Vector3d err_v_inte, delta_err_v, last_err_v;

    Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
    Kd << param_.gain.Kd0, param_.gain.Kd1, param_.gain.Kd2;
    Ki << param_.gain.Ki0, param_.gain.Ki1, param_.gain.Ki2;

    double now = ros::Time::now().toSec();

    static bool lpf_init = false;
    static double alpha  = 0.8;
    static Eigen::Vector3d v_filt;

    if (!lpf_init) {
        v_filt   = odom.v;
        lpf_init = true;
    } else {
        v_filt = alpha * odom.v + (1 - alpha) * v_filt;
    }

    static double last_time;
    static bool first_time = true;
    if (first_time) {
        last_time  = now;
        first_time = false;
    }
    double dt = (now - last_time);
    last_time = now;

    Eigen::Vector3d e_p = des.p - odom.p;
    Eigen::Vector3d e_v = des.v - v_filt;

    // position integral (slow, bounded)
    static Eigen::Vector3d e_v_int(0, 0, 0);
    e_v_int += e_v * dt;
    double i_limit = 1.0;
    e_v_int        = e_v_int.cwiseMax(-i_limit).cwiseMin(i_limit);
    e_v_int *= 0.995;
    e_v_int.z() = 0;

    // final acceleration command
    des_acc = Kp.asDiagonal() * e_p + Kd.asDiagonal() * e_v + Ki.asDiagonal() * e_v_int + des.a +
              Eigen::Vector3d(0, 0, param_.gra);

    double roll, pitch;
    double yaw_odom      = fromQuaternion2yaw(odom.q);
    double sin           = std::sin(yaw_odom);
    double cos           = std::cos(yaw_odom);
    roll                 = (des_acc(0) * sin - des_acc(1) * cos) / param_.gra;
    pitch                = (des_acc(0) * cos + des_acc(1) * sin) / param_.gra;
    Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    u.q = imu.q * odom.q.inverse() * q;
    Eigen::Vector3d des_acc_thr(0.0, 0.0, des_acc(2));
    des_acc_thr(2) = des_acc_thr(2) / (std::cos(pitch) * std::cos(roll));
    u.thrust       = computeDesiredCollectiveThrustSignal(des_acc_thr);

    // used for debug
    debug_msg_.des_v_x = des.v(0);
    debug_msg_.des_v_y = des.v(1);
    debug_msg_.des_v_z = des.v(2);

    debug_msg_.des_a_x = des_acc(0);
    debug_msg_.des_a_y = des_acc(1);
    debug_msg_.des_a_z = des_acc(2);

    debug_msg_.des_q_x = u.q.x();
    debug_msg_.des_q_y = u.q.y();
    debug_msg_.des_q_z = u.q.z();
    debug_msg_.des_q_w = u.q.w();

    debug_msg_.des_thr = u.thrust;

    // Used for thrust-accel mapping estimation
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));

    while (timed_thrust_.size() > 100) {
        timed_thrust_.pop();
    }
    return debug_msg_;
}

/*
  compute throttle percentage
*/
double LinearControl::computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc) {
    double throttle_percentage(0.0);

    /* compute throttle, thr2acc has been estimated before */
    throttle_percentage = des_acc(2) / thr2acc_;

    return throttle_percentage;
}

bool LinearControl::estimateThrustModel(const Eigen::Vector3d &est_a, const Parameter_t &param) {
    ros::Time t_now = ros::Time::now();
    while (timed_thrust_.size() >= 1) {
        // Choose data before 35~45ms ago
        std::pair<ros::Time, double> t_t = timed_thrust_.front();
        double time_passed               = (t_now - t_t.first).toSec();
        if (time_passed > 0.045)  // 45ms
        {
            // printf("continue, time_passed=%f\n", time_passed);
            timed_thrust_.pop();
            continue;
        }
        if (time_passed < 0.035)  // 35ms
        {
            // printf("skip, time_passed=%f\n", time_passed);
            return false;
        }

        /***********************************************************/
        /* Recursive least squares algorithm with vanishing memory */
        /***********************************************************/
        double thr = t_t.second;
        timed_thrust_.pop();

        /***********************************/
        /* Model: est_a(2) = thr1acc_ * thr */
        /***********************************/
        double gamma = 1 / (rho2_ + thr * P_ * thr);
        double K     = gamma * P_ * thr;
        thr2acc_     = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
        P_           = (1 - K * thr) * P_ / rho2_;
        if (param_.thr_map.print_val == true) {
            printf("%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\n", est_a(2), thr, thr2acc_, gamma, K, P_);
            fflush(stdout);
        }

        debug_msg_.thr_scale_compensate = thr2acc_;
        return true;
    }
    return false;
}

bool LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a, const Parameter_t &param, const Battery_Data_t &bat_data) {
    ros::Time t_now = ros::Time::now();

    while (!timed_thrust_.empty()) {
        std::pair<ros::Time, double> t_t = timed_thrust_.front();
        double time_passed               = (t_now - t_t.first).toSec();

        if (time_passed > 0.045) {
            timed_thrust_.pop();
            continue;
        }
        if (time_passed < 0.035) {
            return false;
        }

        double thr = t_t.second;
        timed_thrust_.pop();

        double measured_a = est_a(2);

        if (thr < 0.1) {
            return false;
        }

        double acc_variance = std::abs(measured_a - 9.8);
        if (acc_variance < 0.1) {
            return false;
        }

        double error = measured_a - thr * thr2acc_;

        // Update K (Gain)
        double gamma = 1.0 / (rho2_ + thr * P_ * thr);
        double K     = P_ * thr * gamma;

        // Update Estimate
        thr2acc_ = thr2acc_ + K * error;

        // Update Covariance
        P_ = (P_ - K * thr * P_) / rho2_;

        double min_thr2acc = 10.0;
        double max_thr2acc = 45.0;
        if (thr2acc_ < min_thr2acc) {
            thr2acc_ = min_thr2acc;
        }
        if (thr2acc_ > max_thr2acc) {
            thr2acc_ = max_thr2acc;
        }

        if (P_ > 100.0) {
            P_ = 100.0;
        }
        if (P_ < 0.01) {
            P_ = 0.01;
        }

        debug_msg_.thr_scale_compensate = thr2acc_;
        return true;
    }
    return false;
}

bool LinearControl::estimateThrustModelUsingVelFB(
    const Eigen::Vector3d &est_v, const Parameter_t &param) {
    ros::Time t_now = ros::Time::now();
    while (timed_thrust_.size() >= 1) {
        // Choose data before 35~45ms ago
        std::pair<ros::Time, double> t_t = timed_thrust_.front();
        Eigen::Vector3d t_v              = timed_vel_.front();
        double time_passed               = (t_now - t_t.first).toSec();
        if (time_passed > 0.045)  // 45ms
        {
            // printf("continue, time_passed=%f\n", time_passed);
            timed_thrust_.pop();
            timed_vel_.pop();
            continue;
        }
        if (time_passed < 0.035)  // 35ms
        {
            // printf("skip, time_passed=%f\n", time_passed);
            return false;
        }

        /***********************************************************/
        /* Recursive least squares algorithm with vanishing memory */
        /***********************************************************/
        double thr = t_t.second;
        timed_thrust_.pop();

        /***********************************/
        /* Model: est_a(2) = thr1acc_ * thr */
        /***********************************/
        double gamma = 1 / (rho2_ + thr * P_ * thr);
        double K     = gamma * P_ * thr;
        double est_a = (est_v(2) - t_v(2)) / time_passed;
        thr2acc_     = thr2acc_ + K * (est_a - thr * thr2acc_);
        P_           = (1 - K * thr) * P_ / rho2_;
        // printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
        // fflush(stdout);

        debug_msg_.thr_scale_compensate = thr2acc_;
        return true;
    }
    return false;
}

void LinearControl::resetThrustMapping(void) {
    thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
    P_       = 1e6;
}

void LinearControl::resetThrustMapping(Battery_Data_t &bat_data) {
    printf("Recieve bat volt: %f  V\n", bat_data.volt);

    double volt = bat_data.volt;
    double tmp  = volt2HoverPerOverM0(volt);
    thr2acc_    = param_.gra / (param_.mass * tmp);

    printf("hover percentage: %f\n", param_.mass * tmp);

    P_ = 1e6;
}

double LinearControl::volt2HoverPerOverM0(double volt) {
    double tmp;
    if (volt > 14.6713340547349) {
        tmp = -0.0006 * volt + 0.4067;
        return tmp;
    }
    tmp = 0.0027 * (volt * volt) - 0.1096 * volt + 1.4284;

    return tmp;
}
