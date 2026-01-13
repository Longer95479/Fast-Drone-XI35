#ifndef ZUPT_H
#define ZUPT_H

#include "../estimator/parameters.h"
#include <Eigen/Dense>
#include <algorithm>
#include <deque>
#include <iostream>
#include <map>
#include <mutex>
#include <vector>

class LowPassFilter {
  public:
    LowPassFilter(double sample_freq, double cutoff_freq)
        : sample_freq_hz_(sample_freq), cutoff_freq_hz_(cutoff_freq) {
        double wc = 2.0 * M_PI * cutoff_freq_hz_;
        double T  = 1.0 / sample_freq_hz_;
        double b  = wc * T;
        a1_       = b / (1.0 + b);
    }

    void process(
        const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr, Eigen::Vector3d* acc_filt,
        Eigen::Vector3d* gyr_filt) {
        if (!init_) {
            acc_prev1_      = acc;
            gyr_prev1_      = gyr;
            acc_filt_prev1_ = acc;
            gyr_filt_prev1_ = gyr;
            init_           = true;
            return;
        }

        *acc_filt = a1_ * acc + (1 - a1_) * acc_filt_prev1_;
        *gyr_filt = a1_ * gyr + (1 - a1_) * gyr_filt_prev1_;

        acc_filt_prev1_ = *acc_filt;
        gyr_filt_prev1_ = *gyr_filt;
    }

  private:
    bool is_inited = false;
    double a1_     = 1;
    double sample_freq_hz_;
    double cutoff_freq_hz_;

    bool init_ = false;

    Eigen::Vector3d acc_prev1_;
    Eigen::Vector3d gyr_prev1_;
    Eigen::Vector3d acc_filt_prev1_;
    Eigen::Vector3d gyr_filt_prev1_;
};

struct ZuptResultInfo {
    ZuptResultInfo() {}
    ZuptResultInfo(
        double t, bool is_static, const Eigen::Vector3d& acc_raw, const Eigen::Vector3d& gyr_raw,
        const Eigen::Quaterniond& q_GI_t)
        : t_(t), is_static_(is_static), acc_raw_(acc_raw), gyr_raw_(gyr_raw), q_GI_t_(q_GI_t) {}

    double t_;
    bool is_static_;
    Eigen::Vector3d acc_raw_;
    Eigen::Vector3d gyr_raw_;
    Eigen::Quaterniond q_GI_t_;
};

class Zupt {
  public:
    Zupt() {}

    bool zuptDetection(
        double t, const Eigen::Vector3d* const vel_ptr,
        const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>* const
            features_frame_ptr);

    bool getResultInfo(double t, ZuptResultInfo* info);

    void inputIMU(
        double t, const Eigen::Vector3d& linearAcceleration, const Eigen::Vector3d& angularVelocity,
        const Eigen::Quaterniond& last_Q);

    double inputFeatureAndCheckParallax(
        double t, const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>&
                      features_frame);

    void getIMURawAndQuaternion(
        double t, std::deque<std::pair<double, Eigen::Vector3d>>& acc_raw_buf,
        std::deque<std::pair<double, Eigen::Vector3d>>& gyr_raw_buf,
        std::deque<std::pair<double, Eigen::Quaterniond>>& q_GI_buf, Eigen::Vector3d* acc_raw_ptr,
        Eigen::Vector3d* gyr_raw_ptr, Eigen::Quaterniond* q_GI_ptr);

  private:
    static constexpr double imu_freq_hz_ = 250.0;
    static constexpr double img_freq_hz_ = 10.0;

    static constexpr double gyr_amp_static_thr_ = 0.02;
    static constexpr double acc_amp_static_thr_ = 0.5;

    static constexpr double gyr_avg_static_thr_ = 0.05;
    static constexpr double acc_avg_static_thr_ = 1.5;

    static constexpr double vel_static_thr_      = 0.1;
    static constexpr double parallax_static_thr_ = 5e-4;
    static constexpr double unstatic_ratio_thr_  = 0.05;

    static constexpr int feature_frame_size_ = 10;
    static constexpr int queue_size_ =
        static_cast<int>(imu_freq_hz_ / img_freq_hz_) * feature_frame_size_;
    static constexpr int raw_meas_queue_size_ = static_cast<int>(imu_freq_hz_ / img_freq_hz_);

    static constexpr double lpf_cutoff_freq_hz_ = 1.0;

    static constexpr double eps_ = 1e-4;

    // (timestamp, data)
    std::deque<std::pair<double, Eigen::Vector3d>> acc_buf_, acc_filt_buf_;
    std::deque<std::pair<double, Eigen::Vector3d>> gyr_buf_, gyr_filt_buf_;
    std::deque<
        std::pair<double, std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>>>
        featureBuf_;

    std::deque<std::pair<double, double>> parallax_buf_;

    // (timestamp, newest_measurement) for construct fake meas.
    std::deque<std::pair<double, Eigen::Vector3d>> acc_raw_buf_;
    std::deque<std::pair<double, Eigen::Vector3d>> gyr_raw_buf_;
    std::deque<std::pair<double, Eigen::Quaterniond>> quaternion_GI_buf_;

    std::deque<ZuptResultInfo> result_buf_;

    LowPassFilter lpf_{imu_freq_hz_, lpf_cutoff_freq_hz_};

    std::mutex mBuf;
};

#endif
