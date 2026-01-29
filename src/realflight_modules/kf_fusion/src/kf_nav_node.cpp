#include "kf_nav/kf_interface.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "kf_navigator");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
    if (argc != 2) {
        std::cout << "please input config file!" << std::endl;
        return 1;
    }
    std::string config_file = argv[1];
    std::cout << "config file:" << config_file << std::endl;
    ParamReader::getInstance().initialize(config_file);
    ParamReader::getInstance().printAllParameters();
    KfInterface kf_interface;
    kf_interface.init(nh);

    ros::spin();
    return 0;
}