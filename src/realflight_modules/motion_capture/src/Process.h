#ifndef __PROCESS_H
#define __PROCESS_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mutex>
#include <queue>
#include <ros/ros.h>

extern ros::Publisher pub_mc_odometry;

class Process {
  public:
    Process();

    void run();

    void feed_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void feed_twist(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void feed_accel(const geometry_msgs::TwistStamped::ConstPtr& msg);

  private:
    std::queue<geometry_msgs::PoseStamped> pose;
    std::mutex pose_mutex;
    std::queue<geometry_msgs::TwistStamped> twist;
    std::mutex twist_mutex;
    std::queue<geometry_msgs::TwistStamped> accel;
    std::mutex accel_mutex;
};

#endif