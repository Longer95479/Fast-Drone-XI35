#include <ros/ros.h>
#include "search_plan/search_plan_fsm.h"
using namespace auto_search;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "search_plan_node");
  ros::NodeHandle nh;
  Search_Plan_FSM search_fsm;

  search_fsm.init(nh);
  
  ros::spin();
  return 0;
}