#include "PX4CtrlParam.h"

Parameter_t::Parameter_t() {}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh) {
    read_essential_param(nh, "gain/Kp0", gain.Kp0);
    read_essential_param(nh, "gain/Kp1", gain.Kp1);
    read_essential_param(nh, "gain/Kp2", gain.Kp2);
    read_essential_param(nh, "gain/Ki0", gain.Ki0);
    read_essential_param(nh, "gain/Ki1", gain.Ki1);
    read_essential_param(nh, "gain/Ki2", gain.Ki2);
    read_essential_param(nh, "gain/Kd0", gain.Kd0);
    read_essential_param(nh, "gain/Kd1", gain.Kd1);
    read_essential_param(nh, "gain/Kd2", gain.Kd2);

    read_essential_param(nh, "rotor_drag/x", rt_drag.x);
    read_essential_param(nh, "rotor_drag/y", rt_drag.y);
    read_essential_param(nh, "rotor_drag/z", rt_drag.z);
    read_essential_param(nh, "rotor_drag/k_thrust_horz", rt_drag.k_thrust_horz);

    read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
    read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
    read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
    read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
    read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);

    read_essential_param(nh, "pose_solver", pose_solver);
    read_essential_param(nh, "mass", mass);
    read_essential_param(nh, "gra", gra);
    read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
    read_essential_param(nh, "use_bodyrate_ctrl", use_bodyrate_ctrl);
    read_essential_param(nh, "max_manual_vel", max_manual_vel);
    read_essential_param(nh, "max_angle", max_angle);
    read_essential_param(nh, "low_voltage", low_voltage);
    read_essential_param(nh, "use_motion_capture_odom", use_motion_capture_odom);

    read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
    read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
    read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
    read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);

    read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land.enable);
    read_essential_param(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land.enable_auto_arm);
    read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land.no_RC);
    read_essential_param(nh, "auto_takeoff_land/takeoff_height", takeoff_land.height);
    read_essential_param(nh, "auto_takeoff_land/takeoff_land_speed", takeoff_land.speed);

    read_essential_param(nh, "thrust_model/print_value", thr_map.print_val);
    read_essential_param(nh, "thrust_model/K1", thr_map.K1);
    read_essential_param(nh, "thrust_model/K2", thr_map.K2);
    read_essential_param(nh, "thrust_model/K3", thr_map.K3);
    read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
    read_essential_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);
    read_essential_param(
        nh, "thrust_model/imu_acc_lpf_freq_cutoff", thr_map.imu_acc_lpf_freq_cutoff);

    read_essential_param(nh, "mavros/bat_msg_freq", mavros_bat_msg_freq);
    read_essential_param(nh, "mavros/battery_id", mavros_battery_id);

    max_angle /= (180.0 / M_PI);

    if (takeoff_land.enable_auto_arm && !takeoff_land.enable) {
        takeoff_land.enable_auto_arm = false;
        ROS_ERROR("\"enable_auto_arm\" is only allowd with \"auto_takeoff_land\" enabled.");
    }
    if (takeoff_land.no_RC && (!takeoff_land.enable_auto_arm || !takeoff_land.enable)) {
        takeoff_land.no_RC = false;
        ROS_ERROR(
            "\"no_RC\" is only allowd with both \"auto_takeoff_land\" and \"enable_auto_arm\" "
            "enabled.");
    }

    if (thr_map.print_val) {
        ROS_WARN("You should disable \"print_value\" if you are in regular usage.");
    }
};

// void Parameter_t::config_full_thrust(double hov)
// {
// 	full_thrust = mass * gra / hov;
// };
