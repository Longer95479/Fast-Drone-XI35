#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include "TargetInfoMergedlcm/TargetInfoMerged.hpp"
#include <set>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

#include <ros/ros.h>
#include <lcm_receive/TargetMerged_Message.h> 

std::string message_log_path;

bool lock_file(int fd) {
    struct flock lock;
    lock.l_type = F_WRLCK;
    lock.l_whence = SEEK_SET;
    lock.l_start = 0;
    lock.l_len = 0;
    if (fcntl(fd, F_SETLK, &lock) == -1) {
        return false;
    }
    return true;
}

void unlock_file(int fd) {
    struct flock lock;
    lock.l_type = F_UNLCK;
    lock.l_whence = SEEK_SET;
    lock.l_start = 0;
    lock.l_len = 0;
    fcntl(fd, F_SETLK, &lock);
}

bool load_from_file(std::set<std::pair<int64_t, int>>& logged_messages) {
    logged_messages.clear();
    int fd = open(message_log_path.c_str(), O_RDWR | O_CREAT | O_APPEND, 0644);
    if (fd == -1) {
        perror("open");
        return false;
    }

    if (!lock_file(fd)) {
        close(fd);
        return false;
    }

    std::ifstream file(message_log_path.c_str());
    if (file.is_open()) {
        int64_t timestamp;
        int drone_id;
        while (file >> timestamp >> drone_id) {
            logged_messages.insert(std::make_pair(timestamp, drone_id));
        }
        file.close();
    } else {
        close(fd);
        unlock_file(fd);
        return false;
    }

    unlock_file(fd);
    close(fd);
    return true;
}

bool append_to_file(const std::pair<int64_t, int>& new_entry) {
    int fd = open(message_log_path.c_str(), O_RDWR | O_CREAT | O_APPEND, 0644);
    if (fd == -1) {
        perror("open");
        return false;
    }

    if (!lock_file(fd)) {
        close(fd);
        return false;
    }

    std::ofstream file(message_log_path.c_str(), std::ios::app);
    if (file.is_open()) {
        file << new_entry.first << " " << new_entry.second << "\n";
        file.close();
    } else {
        close(fd);
        unlock_file(fd);
        return false;
    }

    unlock_file(fd);
    close(fd);
    return true;
}

class Handler {
public:
    Handler(lcm::LCM& lcm, ros::Publisher& ros_pub) : lcm(lcm), ros_pub(ros_pub) {}

    ~Handler() {}

    void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const TargetInfoMergedlcm::TargetInfoMerged* msg) {
        std::set<std::pair<int64_t, int>> logged_messages;
        bool file_loaded = load_from_file(logged_messages);

        if (!file_loaded) {
            printf("Message will not be processed.\n");
            return;
        }

        auto entry = std::make_pair(msg->timestamp, msg->drone_id);
        if (logged_messages.find(entry) == logged_messages.end()) {

            if (!append_to_file(entry)) {
                printf("Message will not be processed.\n");
                return;
            }

            lcm_receive::TargetMerged_Message ros_msg;
            ros_msg.header.stamp = ros::Time::now();
            ros_msg.time = msg->time;
            ros_msg.id = msg->drone_id;
            ros_msg.type = msg->target_category;
            ros_msg.x = msg->position[0];
            ros_msg.y = msg->position[1];
            ros_msg.cov[0] = msg->variance[0];
            ros_msg.cov[1] = msg->variance[1];
            ros_msg.cov[2] = msg->variance[2];
            ros_msg.cov[3] = msg->variance[3];

            ros_pub.publish(ros_msg);

            TargetInfoMergedlcm::TargetInfoMerged new_data;
            new_data.timestamp = msg->timestamp; 
            new_data.time = msg->time;
            new_data.drone_id = msg->drone_id;  
            new_data.target_category = msg->target_category;  
            new_data.position[0] = msg->position[0]; 
            new_data.position[1] = msg->position[1];
            new_data.variance[0] = msg->variance[0];
            new_data.variance[1] = msg->variance[1];
            new_data.variance[2] = msg->variance[2];
            new_data.variance[3] = msg->variance[3];

            lcm.publish("TARGETINFO_MERGED", &new_data);
        }
    }

private:
    lcm::LCM& lcm;
    ros::Publisher& ros_pub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lcm_receive_node");
    ros::NodeHandle nh;
    nh.getParam("/lcm_receive/message_log_path", message_log_path);
    ros::Publisher ros_pub = nh.advertise<lcm_receive::TargetMerged_Message>("/communication/sub_target_merged", 10);

    lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm.good())
        return 1;

    Handler handlerObject(lcm, ros_pub);
    lcm.subscribe("TARGETINFO_MERGED", &Handler::handleMessage, &handlerObject);

    while (ros::ok()) {
        lcm.handle();
        ros::spinOnce();
    }

    return 0;
}
