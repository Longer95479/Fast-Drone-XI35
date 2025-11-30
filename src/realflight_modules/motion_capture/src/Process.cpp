#include "Process.h"
#include <nav_msgs/Odometry.h>

Process::Process() {}

void Process::feed_pose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(pose_mutex);
    pose.push(*msg);
}

void Process::feed_twist(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(twist_mutex);
    twist.push(*msg);
}

void Process::feed_accel(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(accel_mutex);
    accel.push(*msg);
}

geometry_msgs::TwistStamped get_insert_data(
    std::queue<geometry_msgs::TwistStamped>& msg, const ros::Time& insert_stamp) {
    if (msg.empty()) {
        return geometry_msgs::TwistStamped();
    }

    geometry_msgs::TwistStamped cur_data;
    geometry_msgs::TwistStamped next_data;
    geometry_msgs::TwistStamped insert_data;
    while (!msg.empty() && msg.front().header.stamp < insert_stamp) {
        cur_data = msg.front();

        msg.pop();
        if (msg.empty()) {
            return geometry_msgs::TwistStamped();
        }
        next_data = msg.front();
    }

    double t_a = cur_data.header.stamp.toSec();
    double t_b = next_data.header.stamp.toSec();
    double t   = insert_stamp.toSec();

    if (fabs(t_b - t_a) < 1e-12) {
        return cur_data;
    }

    double ratio = (t - t_a) / (t_b - t_a);

    insert_data.twist.linear.x =
        cur_data.twist.linear.x + (next_data.twist.linear.x - cur_data.twist.linear.x) * ratio;
    insert_data.twist.linear.y =
        cur_data.twist.linear.y + (next_data.twist.linear.y - cur_data.twist.linear.y) * ratio;
    insert_data.twist.linear.z =
        cur_data.twist.linear.z + (next_data.twist.linear.z - cur_data.twist.linear.z) * ratio;

    insert_data.twist.angular.x =
        cur_data.twist.angular.x + (next_data.twist.angular.x - cur_data.twist.angular.x) * ratio;
    insert_data.twist.angular.y =
        cur_data.twist.angular.y + (next_data.twist.angular.y - cur_data.twist.angular.y) * ratio;
    insert_data.twist.angular.z =
        cur_data.twist.angular.z + (next_data.twist.angular.z - cur_data.twist.angular.z) * ratio;

    insert_data.header.frame_id = cur_data.header.frame_id;
    insert_data.header.stamp    = insert_stamp;
    insert_data.header.seq      = cur_data.header.seq;

    return insert_data;
}

void Process::run() {
    std::lock_guard<std::mutex> lock_pose(pose_mutex);
    std::lock_guard<std::mutex> lock_twist(twist_mutex);
    std::lock_guard<std::mutex> lock_accel(accel_mutex);
    if (pose.empty() || twist.empty() || accel.empty()) {
        return;
    }

    while (pose.front().header.stamp <= twist.front().header.stamp ||
           pose.front().header.stamp <= accel.front().header.stamp) {
        pose.pop();
        if (pose.empty()) {
            return;
        }
    }

    geometry_msgs::TwistStamped twist_data = get_insert_data(twist, pose.front().header.stamp);

    if (twist_data.header.stamp.isZero()) {
        return;
    }
    geometry_msgs::TwistStamped accel_data = get_insert_data(accel, pose.front().header.stamp);
    if (accel_data.header.stamp.isZero()) {
        return;
    }

    nav_msgs::Odometry mc_odometry;
    mc_odometry.header          = pose.front().header;
    mc_odometry.header.frame_id = "world";
    mc_odometry.child_frame_id  = "world";

    mc_odometry.header.stamp            = pose.front().header.stamp;
    mc_odometry.header.frame_id         = "world";
    mc_odometry.pose.pose.position.x    = pose.front().pose.position.x;  // position
    mc_odometry.pose.pose.position.y    = pose.front().pose.position.y;
    mc_odometry.pose.pose.position.z    = pose.front().pose.position.z;
    mc_odometry.pose.pose.orientation.x = pose.front().pose.orientation.x;  // orientation
    mc_odometry.pose.pose.orientation.y = pose.front().pose.orientation.y;
    mc_odometry.pose.pose.orientation.z = pose.front().pose.orientation.z;
    mc_odometry.pose.pose.orientation.w = pose.front().pose.orientation.w;
    mc_odometry.twist.twist.linear.x    = twist_data.twist.linear.x;  // linear velocity
    mc_odometry.twist.twist.linear.y    = twist_data.twist.linear.y;
    mc_odometry.twist.twist.linear.z    = twist_data.twist.linear.z;
    mc_odometry.twist.twist.angular.x   = twist_data.twist.angular.x;  // angular velocity
    mc_odometry.twist.twist.angular.y   = twist_data.twist.angular.y;
    mc_odometry.twist.twist.angular.z   = twist_data.twist.angular.z;

    double ax, ay, az;              // linear acceleration
    double alphax, alphay, alphaz;  // angular acceleration
    ax     = accel_data.twist.linear.x;
    ay     = accel_data.twist.linear.y;
    az     = accel_data.twist.linear.z;
    alphax = accel_data.twist.angular.x;
    alphay = accel_data.twist.angular.y;
    alphaz = accel_data.twist.angular.z;
    // [ ax ay az alphax alphay alphaz ]
    mc_odometry.twist.covariance[0] = ax;
    mc_odometry.twist.covariance[1] = ay;
    mc_odometry.twist.covariance[2] = az;
    mc_odometry.twist.covariance[3] = alphax;
    mc_odometry.twist.covariance[4] = alphay;
    mc_odometry.twist.covariance[5] = alphaz;

    pub_mc_odometry.publish(mc_odometry);
}