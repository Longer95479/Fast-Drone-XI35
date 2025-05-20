#include <ros/ros.h>
#include <lcm_node/CollaSignal.h>
#include <lcm_node/TakeoffLand.h>
#include <lcm/lcm-cpp.hpp>
#include "StartStageCollaboration/CollaSignal.hpp"
#include "TakeoffLand/TakeoffLand.hpp"

class LcmNode {
public:
    LcmNode(ros::NodeHandle& nh, lcm::LCM& lcm) : nh_(nh), lcm_(lcm) {
        ros_sub_ = nh_.subscribe("/start_stage_colla/send_colla_signal", 10, &LcmNode::rosCallback, this);
        ros_pub_ = nh_.advertise<lcm_node::CollaSignal>("/start_stage_colla/receive_colla_signal", 10);
        lcm_.subscribe("colla_signal", &LcmNode::lcmCallback, this);
        ros_takeoff_pub_ = nh_.advertise<lcm_node::TakeoffLand>("/px4ctrl/takeoff_land", 10);
        lcm_.subscribe("takeoff_land", &LcmNode::lcmTakeoffLandCallback, this);
    }

    void rosCallback(const lcm_node::CollaSignal::ConstPtr& msg) {
        StartStageCollaboration::CollaSignal lcm_msg;
        lcm_msg.timestamp = msg->timestamp;
        lcm_msg.from_id = msg->from_id;
        lcm_msg.to_id = msg->to_id;
        lcm_msg.signal_name = msg->signal_name;
        lcm_msg.is_ready = msg->is_ready;
        lcm_.publish("colla_signal", &lcm_msg);
        ROS_INFO("[ROS->LCM] - From: %d, To: %d, Signal Name: %s, Ready: %d", msg->from_id, msg->to_id, msg->signal_name.c_str(), msg->is_ready);
    }

    void lcmCallback(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const StartStageCollaboration::CollaSignal* msg) {
        lcm_node::CollaSignal ros_msg;
        ros_msg.timestamp = msg->timestamp;
        ros_msg.from_id = msg->from_id;
        ros_msg.to_id = msg->to_id;
        ros_msg.signal_name = msg->signal_name;
        ros_msg.is_ready = msg->is_ready;
        ros_pub_.publish(ros_msg);
        ROS_INFO("[LCM->ROS] - From: %d, To: %d, Signal Name: %s, Ready: %d", msg->from_id, msg->to_id, msg->signal_name.c_str(), msg->is_ready);
    }

    void lcmTakeoffLandCallback(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const TakeoffLand::TakeoffLand* msg) {
        lcm_node::TakeoffLand ros_msg;
        ros_msg.takeoff_land_cmd = static_cast<uint8_t>(msg->takeoffland);
        ros_takeoff_pub_.publish(ros_msg);
        ROS_INFO("[LCM->ROS] Takeoff/Land Command: %d", msg->takeoffland);
    }

private:
    ros::NodeHandle nh_;
    lcm::LCM& lcm_;
    ros::Subscriber ros_sub_;
    ros::Publisher ros_pub_;
    ros::Publisher ros_takeoff_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lcm_node");
    ros::NodeHandle nh;
    lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=1");
    LcmNode node(nh, lcm);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    while (ros::ok()) {
        lcm.handleTimeout(10);
    }
    return 0;
}
