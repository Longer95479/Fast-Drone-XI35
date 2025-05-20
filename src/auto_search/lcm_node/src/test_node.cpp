#include <ros/ros.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <lcm_node/CollaSignal.h>

#include <lcm/lcm-cpp.hpp>
#include "StartStageCollaboration/CollaSignal.hpp"

class LCMHandler {
public:
  ~LCMHandler () {}
  void lcmCallBack( const lcm::ReceiveBuffer* rbuf, 
                    const std::string& channel, 
                    const StartStageCollaboration::CollaSignal* msg ) {
    ROS_INFO("[LCM->ROS] - From: %d, To: %d, Signal Name: %s, Ready: %d", 
        msg->from_id, msg->to_id, msg->signal_name.c_str(), msg->is_ready);
  }
};



int main(int argc, char** argv) {
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=1");
  LCMHandler handler_obj;
  lcm.subscribe("colla_signal", &LCMHandler::lcmCallBack, &handler_obj);
  ros::Publisher pub_trigger = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 1);
  ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/vins_fusion/imu_propagate", 1);

  ros::Duration(1).sleep();

  // trigger
  geometry_msgs::PoseStamped trigger_msg;
  pub_trigger.publish(trigger_msg);

  ros::Duration(2).sleep();

  // odom takeoff point
  nav_msgs::Odometry odom_msg;
  odom_msg.pose.pose.position.x = 0.0;
  odom_msg.pose.pose.position.y = 0.0;
  odom_msg.pose.pose.position.z = 0.7;
  pub_odom.publish(odom_msg);

  ros::Duration(2).sleep();

  // odom start point 
  odom_msg.pose.pose.position.x = 1.0;
  odom_msg.pose.pose.position.y = 0.0;
  odom_msg.pose.pose.position.z = 0.7;
  pub_odom.publish(odom_msg);

  // 3->2 HV
  StartStageCollaboration::CollaSignal lcm_msg;
  lcm_msg.timestamp = ros::Time::now().toSec();
  lcm_msg.from_id = 3;
  lcm_msg.to_id = 2;
  lcm_msg.signal_name = "HV";
  lcm_msg.is_ready = true;
  lcm.publish("colla_signal", &lcm_msg);

  ros::Duration(1).sleep();

  lcm_msg.timestamp = ros::Time::now().toSec();
  lcm_msg.from_id = 1;
  lcm_msg.to_id = 2;
  lcm_msg.signal_name = "FW";
  lcm_msg.is_ready = true;
  lcm.publish("colla_signal", &lcm_msg);

  ros::spin();
  while (ros::ok()) {
      lcm.handleTimeout(10);
  }

  return 0;
}

