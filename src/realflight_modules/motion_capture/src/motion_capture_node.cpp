#include "Parameters.h"
#include "Process.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

ros::Publisher pub_mc_odometry;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "motion_capture");
    ros::NodeHandle nh("~");

    Parameter_t param;
    param.config_from_ros_handle(nh);

    Process process;

    ros::Subscriber pose_n = nh.subscribe<geometry_msgs::PoseStamped>(
        param.mc_pose_topic, 100, boost::bind(&Process::feed_pose, &process, _1));
    ros::Subscriber twist_n = nh.subscribe<geometry_msgs::TwistStamped>(
        param.mc_twist_topic, 100, boost::bind(&Process::feed_twist, &process, _1));
    ros::Subscriber accel_n = nh.subscribe<geometry_msgs::TwistStamped>(
        param.mc_accel_topic, 100, boost::bind(&Process::feed_accel, &process, _1));

    pub_mc_odometry = nh.advertise<nav_msgs::Odometry>("motion_capture_odom", 100);

    ros::Rate loop_rate(200);

    while (ros::ok()) {
        process.run();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
