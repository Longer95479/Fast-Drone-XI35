#include <lcm/lcm-cpp.hpp>
#include "TargetInfoMergedlcm/TargetInfoMerged.hpp"
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <chrono>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <lcm_send/TargetMerged_Message.h>

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

bool save_to_file(int64_t timestamp, int drone_id) {
    int fd = open(message_log_path.c_str(), O_RDWR | O_CREAT | O_APPEND, 0644);
    if (fd == -1) {
        perror("open");
        return false;
    }

    if (!lock_file(fd)) {
        close(fd);
        return false;
    }

    std::ofstream file;
    file.open(message_log_path.c_str(), std::ios::app);
    if (file.is_open()) {
        std::cout << timestamp << " " << drone_id << std::endl;
        file << timestamp << " " << drone_id << "\n";
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

int64_t rosTimeToLCM(int32_t sec, uint32_t nsec) {
    return static_cast<int64_t>(sec) * 1000000000 + nsec;
}

void rosCallback(const lcm_send::TargetMerged_Message::ConstPtr& msg, lcm::LCM& lcm) {
    TargetInfoMergedlcm::TargetInfoMerged my_data;
    my_data.timestamp = rosTimeToLCM(msg->header.stamp.sec, msg->header.stamp.nsec);
    my_data.time = msg->time;
    my_data.drone_id = msg->id;
    my_data.target_category = msg->type;
    my_data.position[0] = msg->x;
    my_data.position[1] = msg->y;
    my_data.variance[0] = msg->cov[0];
    my_data.variance[1] = msg->cov[1];
    my_data.variance[2] = msg->cov[2];
    my_data.variance[3] = msg->cov[3];

    bool file_saved = save_to_file(my_data.timestamp, my_data.drone_id);
    if (!file_saved) {
        printf("Message will not be published.\n");
        return;
    }

    lcm.publish("TARGETINFO_MERGED", &my_data);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lcm_send_node");
    ros::NodeHandle nh;
    nh.getParam("/lcm_send/message_log_path", message_log_path);
    lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=1");
    ros::Subscriber sub = nh.subscribe<lcm_send::TargetMerged_Message>("/target_merge/pub_target_merged", 10,
        [&lcm](const lcm_send::TargetMerged_Message::ConstPtr& msg) {
            rosCallback(msg, lcm);
        });
    if (!lcm.good()) {
        return 1;
    }

    ros::spin();

    return 0;
}
