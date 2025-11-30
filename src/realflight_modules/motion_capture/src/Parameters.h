#ifndef __PARAMETERS_H
#define __PARAMETERS_H

#include <ros/ros.h>

class Parameter_t {
  public:
    std::string mc_pose_topic;
    std::string mc_twist_topic;
    std::string mc_accel_topic;

    Parameter_t();
    void config_from_ros_handle(const ros::NodeHandle &nh);

  private:
    template <typename TName, typename TVal>
    void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val) {
        if (nh.getParam(name, val)) {
            // pass
        } else {
            ROS_ERROR_STREAM("Read param: " << name << " failed.");
            ROS_BREAK();
        }
    };
};

#endif