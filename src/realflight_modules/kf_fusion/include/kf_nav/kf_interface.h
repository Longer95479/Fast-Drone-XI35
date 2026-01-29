#pragma once

#include "kf_nav/health_monitor.h"
#include "kf_nav/imu_helper.h"
#include "kf_nav/kf_coordinator.h"
#include "kf_nav/safe_queue.h"
#include <condition_variable>
#include <geometry_msgs/PointStamped.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <thread>

class KfInterface {
  public:
    struct KfInterfaceConfig {
        std::string imu_topic;
        std::string odom_topic;
        std::string vins_invalid_topic;
        bool use_motion_capture;
        void loadConfig() {
            imu_topic          = ParamReader::getInstance().getString("imu_topic", "/imu/data");
            use_motion_capture = ParamReader::getInstance().getBool("use_motion_capture", false);
            vins_invalid_topic = ParamReader::getInstance().getString(
                "vins_valid_topic", "/vins_invalid_topic/data");
            if (use_motion_capture) {
                odom_topic = ParamReader::getInstance().getString(
                    "motion_capture_topic", "/motion_capture/data");
            } else {
                odom_topic = ParamReader::getInstance().getString("odom_topic", "/odom/data");
            }
        }
    };
    KfInterface() {
        config_.loadConfig();
        thrd_hdl_ = std::thread(&KfInterface::processMeasurements, this);
    }
    ~KfInterface() {
        if (thrd_hdl_.joinable()) {
            thrd_hdl_.join();
        }
    }
    void init(ros::NodeHandle& nh);

  protected:
    void receiveImuTopic(const sensor_msgs::ImuConstPtr& imu_msg);
    void receiveOdomTopic(const nav_msgs::OdometryConstPtr& odom_msg);
    void receiveVinsInvalidTopic(const std_msgs::BoolConstPtr& vins_invalid_msg);
    void processMeasurements();
    void publishImuOdom(double t);
    void publishObvOdom(const OdomMeasPtr& odom_meas);
    void publishStatic(bool is_static);
    void publishExt(double t);

  protected:
    KfInterfaceConfig config_;

    ros::Publisher pub_imu_odom_;
    ros::Publisher pub_imu_path_;
    ros::Publisher pub_obv_odom_;
    ros::Publisher pub_obv_path_;
    ros::Publisher pub_static_;
    ros::Publisher pub_ext_;
    ros::Publisher pub_kf_fail_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_vins_fail_;
    nav_msgs::Path imu_path_, obv_path_;

    std::thread thrd_hdl_;
    std::condition_variable cv_msg_;
    std::mutex mtx_msg_;

    bool is_vins_invalid_{false};

    KfCoordinator kf_coordiantor_;
    StateInitializer state_initializer_;
    StaticCheck static_check_;
    ThreadsafeQueue<MeasureBasePtr> meas_que_;
    HealthMonitor health_monitor_;
};