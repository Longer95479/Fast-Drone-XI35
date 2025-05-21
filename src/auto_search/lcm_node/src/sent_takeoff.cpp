#include <ros/ros.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <lcm/lcm-cpp.hpp>
#include "TakeoffLand/TakeoffLand.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "sent_takeoff_signal");
  ros::NodeHandle nh;

  lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=1");

  for (int i = 0; i < 5; i++) {
    TakeoffLand::TakeoffLand lcm_msg;
    lcm_msg.takeoffland = 1;
    lcm.publish("takeoff_land", &lcm_msg);

    ros::Duration(1).sleep();
  }

  ros::shutdown();

  return 0;
}

