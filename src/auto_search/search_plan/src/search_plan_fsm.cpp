#include <string>
#include "search_plan_fsm.h"

namespace auto_search
{
std::string odom_Topic;
double exec_Frequency;
//执行起始阶段状态机
void Search_Plan_FSM::execStartStage()
{

}

//执行搜索阶段状态机
void Search_Plan_FSM::execSearchStage()
{
  
}

//执行降落阶段状态机
void Search_Plan_FSM::execLandStage()
{

}
//状态机执行函数
void Search_Plan_FSM::execFSMCallback(const ros::TimerEvent &e)
{
  timer_FSM.stop();
  switch (main_State)
  {
  case STARTING_AREA_STAGE:
  {
    execStartStage();
    break;
  }
  case SEARCH_STAGE:
  {
    execSearchStage();
    break;
  }
  case LANDING_STAGE:
  {
    execLandStage();
    break;
  }
  default:
    break;
  }
  timer_FSM.start();
}
void Search_Plan_FSM::updateOdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  current_Position[0] = msg->pose.pose.position.x;
  current_Position[1] = msg->pose.pose.position.y;
  current_Position[2] = msg->pose.pose.position.z;
}
void Search_Plan_FSM::publishTarget()
{
  if(current_Target != last_Target)
  {

    last_Target = current_Target;
  }

}
void Search_Plan_FSM::init(ros::NodeHandle& nh)
{
  //param
  nh.param<std::string>("odom_topic", odom_Topic, "/vins/imu_prepagate");
  nh.param<double>("exec_frequency", exec_Frequency, 100);
  //sub
  sub_Odom = nh.subscribe(odom_Topic, 100, &Search_Plan_FSM::updateOdomCallback, this);
  //pub
  pub_Target = nh.advertise<geometry_msgs::PoseStamped>("/auto_search/pos_cmd", 50);
  //timer
  timer_FSM = nh.createTimer(ros::Duration(1/exec_Frequency), &Search_Plan_FSM::execFSMCallback, this);
}
//
}