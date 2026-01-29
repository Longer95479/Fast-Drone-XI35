#pragma once
#include <mutex>

class Estimator;
class FeatureManager;

class HealthMonitor {
  public:
    static constexpr double maxHealthValue = 2.5;
    static constexpr int visFeatCntMax     = 60;
    static constexpr double maxTranslation = 1e4;
    static constexpr double maxVelocity    = 50;

    void init(Estimator* estimator, FeatureManager* feature_manager) {
        estimator_ptr_    = estimator;
        feat_manager_ptr_ = feature_manager;
        health_val_       = 0.8 * maxHealthValue;
        ts_last_imu_prop_ = -1;
        ts_last_upt_      = -1;
        is_init_          = true;
    }
    bool isInit() { return is_init_; }
    void updateFromImuProp(double ts);
    void updateFromVisUpt(double ts);
    bool isVINSHealthy();
    double getHealthPercent() { return health_val_ < 0 ? 0 : health_val_ / maxHealthValue; }
    void CheckAndPublish(ros::Publisher& pub_fail);

  protected:
    Estimator* estimator_ptr_         = nullptr;
    FeatureManager* feat_manager_ptr_ = nullptr;

    std::mutex mtx_val;
    double health_val_;
    double ts_last_imu_prop_;
    double ts_last_upt_;

    bool is_init_{false};
};