#include <ros/ros.h>

#include "kf_nav/ext_calib.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ex_calib");
  ros::NodeHandle nh("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);
  if (argc != 2) {
    std::cout << "please input config file!" << std::endl;
    return 1;
  }
  std::string config_file = argv[1];
  std::cout << "config file:" << config_file << std::endl;
  ParamReader::getInstance().initialize(config_file);
  ExtCalib ext_calib;
  ext_calib.init(nh);
  ros::spin();
  return 0;
}