#ifndef __INPUT_H
#define __INPUT_H

#include "PX4CtrlParam.h"
#include <Eigen/Dense>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <uav_utils/utils.h>

class RC_Data_t {
  public:
    RC_Data_t(const Parameter_t &param)
        : last_mode(-1.0),
          last_gear(-1.0),
          last_reboot_cmd(0.0),
          have_init_last_mode(false),
          have_init_last_gear(false),
          have_init_last_reboot_cmd(false),
          rcv_stamp(ros::Time(0)),
          is_command_mode(true),
          enter_command_mode(false),
          is_hover_mode(true),
          enter_hover_mode(false),
          toggle_reboot(false),
          param_(param) {}

    double mode;
    double gear;
    double reboot_cmd;
    double last_mode;
    double last_gear;
    double last_reboot_cmd;
    bool have_init_last_mode{false};
    bool have_init_last_gear{false};
    bool have_init_last_reboot_cmd{false};
    double ch[4]{0.0, 0.0, 0.0, 0.0};

    mavros_msgs::RCIn msg;
    ros::Time rcv_stamp;

    bool is_command_mode;
    bool enter_command_mode;
    bool is_hover_mode;
    bool enter_hover_mode;
    bool toggle_reboot;

    static constexpr double GEAR_SHIFT_VALUE         = 0.75;
    static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;
    static constexpr double REBOOT_THRESHOLD_VALUE   = 0.5;
    static constexpr double DEAD_ZONE                = 0.25;

    void check_validity();
    bool check_centered();
    void feed(mavros_msgs::RCInConstPtr pMsg);
    bool is_received(const ros::Time &now_time);

  private:
    const Parameter_t &param_;
};

class Odom_Data_t {
  public:
    Odom_Data_t(const Parameter_t &param)
        : q(Eigen::Quaterniond::Identity()),
          rcv_stamp(ros::Time(0)),
          recv_new_msg(false),
          param_(param) {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    Eigen::Vector3d w;

    nav_msgs::Odometry msg;
    ros::Time rcv_stamp;
    bool recv_new_msg;

    void feed(nav_msgs::OdometryConstPtr pMsg);

    bool is_received(const ros::Time &now_time);

  private:
    const Parameter_t &param_;
};

class Imu_Data_t {
  public:
    Imu_Data_t(const Parameter_t &param) : rcv_stamp(ros::Time(0)), param_(param) {}
    Eigen::Quaterniond q;
    Eigen::Vector3d w;
    Eigen::Vector3d a;

    sensor_msgs::Imu msg;
    ros::Time rcv_stamp;

    void feed(sensor_msgs::ImuConstPtr pMsg);
    bool is_received(const ros::Time &now_time);

  private:
    const Parameter_t &param_;
};

class State_Data_t {
  public:
    mavros_msgs::State current_state;
    mavros_msgs::State state_before_offboard;

    State_Data_t();
    void feed(mavros_msgs::StateConstPtr pMsg);
};

class ExtendedState_Data_t {
  public:
    mavros_msgs::ExtendedState current_extended_state;

    ExtendedState_Data_t();
    void feed(mavros_msgs::ExtendedStateConstPtr pMsg);
};

class Command_Data_t {
  public:
    Command_Data_t(const Parameter_t &param) : rcv_stamp(ros::Time(0)), param_(param) {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    double yaw;
    double yaw_rate;

    quadrotor_msgs::PositionCommand msg;
    ros::Time rcv_stamp;

    void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
    bool is_received(const ros::Time &now_time);

  private:
    const Parameter_t &param_;
};

class Battery_Data_t {
  public:
    Battery_Data_t(const Parameter_t &param) : rcv_stamp(ros::Time(0)), param_(param) {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double volt{0.0};
    double percentage{0.0};

    sensor_msgs::BatteryState msg;
    ros::Time rcv_stamp;

    void feed(sensor_msgs::BatteryStateConstPtr pMsg);
    bool is_received(const ros::Time &now_time);

  private:
    const Parameter_t &param_;
};

class Takeoff_Land_Data_t {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool triggered{false};
    uint8_t takeoff_land_cmd;  // see TakeoffLand.msg for its defination

    quadrotor_msgs::TakeoffLand msg;
    ros::Time rcv_stamp;

    Takeoff_Land_Data_t();
    void feed(quadrotor_msgs::TakeoffLandConstPtr pMsg);
};

#endif
