#include "Parameters.h"

Parameter_t::Parameter_t() {}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh) {
    read_essential_param(nh, "motion_capture_pose_topic", mc_pose_topic);
    read_essential_param(nh, "motion_capture_twist_topic", mc_twist_topic);
    read_essential_param(nh, "motion_capture_accel_topic", mc_accel_topic);
};
