#pragma once
#include "kf_nav/kf_coordinator.h"
#include <mutex>

class HealthMonitor {
  public:
    static constexpr double maxHealthValue        = 3.0;
    static constexpr double mc_update_fre         = 90;
    static constexpr double vis_update_fre        = 10;
    static constexpr double mc_update_increment   = 2 / mc_update_fre;
    static constexpr double vis_update_increment  = 2 / vis_update_fre;
    static constexpr double zupt_update_increment = 0.1;
    static constexpr double maxTranslation        = 1e4;
    static constexpr double maxVelocity           = 50;

    void init(KfCoordinator* kf_coordiantor_ptr) {
        kf_coordiantor_ptr_ = kf_coordiantor_ptr;
        health_val_         = 0.8 * maxHealthValue;
        ts_last_imu_prop_   = -1;
        is_init_            = true;
        is_vins_invalid_    = false;
    }
    bool isInit() { return is_init_; }
    void updateFromImuProp(double ts);
    void updateFromVisUpt(double ts);
    void updateFromMcUpt(double ts);
    void updateFromZupt(double ts);
    void setVinsInvalid(bool is_invalid) { is_vins_invalid_ = is_invalid; }
    bool isKfHealthy();
    double getHealthPercent() { return health_val_ < 0 ? 0 : health_val_ / maxHealthValue; }
    void CheckAndPublish(ros::Publisher& pub_fail);

  protected:
    KfCoordinator* kf_coordiantor_ptr_ = nullptr;

    std::mutex mtx_val;
    double health_val_;
    double ts_last_imu_prop_;

    bool is_init_{false};
    bool is_vins_invalid_{true};
};