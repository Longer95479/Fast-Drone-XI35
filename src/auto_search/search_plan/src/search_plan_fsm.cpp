#include <string>
#include "search_plan/search_plan_fsm.h"

namespace auto_search
{

//执行起始阶段状态机
void Search_Plan_FSM::execStartStage()
{
  switch (start_SubState)
  {
  case TAKE_OFF:
  {
    if(have_trigger)
    {
      changeStartSubState(FLY_TO_START);
      ROS_INFO("Start substate switch: TAKEOFF ======> FLY_TO_START");
    }
    break;
  }
  case FLY_TO_START:
  {
    current_Target = search_StartPoint;
    if(haveArrivedTarget())
    {
      changeStartSubState(WAIT_FOR_START);
      ROS_INFO("Start substate switch: FLY_TO_START ======> WAIT_FOR_START");
    }
    break;
  }
  case WAIT_FOR_START:
  {
    //todo 所有无人机就绪后再进入搜索阶段
    changeMainState(SEARCH_STAGE);
    ROS_INFO("Main state switch: STARTING_AREA_STAGE ======> SEARCH_STAGE");
    break;
  }
  }
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
  case LAND_STAGE:
  {
    execLandStage();
    break;
  }
  default:
    break;
  }
  publishTarget();
  timer_FSM.start();
}
void Search_Plan_FSM::updateOdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  current_Position[0] = msg->pose.pose.position.x;
  current_Position[1] = msg->pose.pose.position.y;
  current_Position[2] = msg->pose.pose.position.z;
}
void Search_Plan_FSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
{
  have_trigger= true;
  std::cout << "Triggered!" << std::endl;
}
void Search_Plan_FSM::publishTarget()
{
  double dis = (current_Target - last_Target).norm();
  if( dis >= 0.2)//新的目标点距离之前的目标点大于0.2m才发布新的目标点
  {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = current_Position.x();
    msg.pose.position.y = current_Position.y();
    msg.pose.position.z = current_Position.z();
    pub_Target.publish(msg);

    last_Target = current_Target;
  }
}

void Search_Plan_FSM::changeMainState(MAIN_STATE next_state)
{
  main_State = next_state;
}
void Search_Plan_FSM::changeStartSubState(START_SUB_STATE next_state)
{
  start_SubState = next_state;
}
void Search_Plan_FSM::changeSearchSubState(SEARCH_SUB_STATE next_state)
{
  search_SubState = next_state;
}
void Search_Plan_FSM::changeLandSubState(LAND_SUB_STATE next_state)
{
  land_SubState = next_state;
}

bool Search_Plan_FSM::haveArrivedTarget()
{
  double distance = (current_Target - current_Position).norm();
  if(distance <= arrive_Threshold)
    return true;
  return false;
}
//初始化各状态
void Search_Plan_FSM::initState()
{
  main_State = STARTING_AREA_STAGE;
  start_SubState = TAKE_OFF;
  search_SubState = SEARCH_NUMS;
  land_SubState = FLY_TO_NUMS;
  ROS_INFO("Initail State: STARTING_AREA_STAGE<TAKE_OFF>");
}

void Search_Plan_FSM::init(ros::NodeHandle& nh)
{
  //var init
  initState();
  current_Target << 0.0, 0.0, 0.0;
  last_Target << 0.0, 0.0, 0.0;

  //param
  nh.param<std::string>("odom_topic", odom_Topic, "/vins_fusion/imu_propagate");
  nh.param<double>("exec_frequency", exec_Frequency, 100);
  nh.param<double>("arrive_threshold", arrive_Threshold, 0.3);
  
  nh.param<double>("search_startpoint_x", search_StartPoint.x(), 2);
  nh.param<double>("search_startpoint_y", search_StartPoint.y(), 0.5);
  nh.param<double>("search_startpoint_z", search_StartPoint.z(), 2);

  //sub
  sub_Odom = nh.subscribe(odom_Topic, 1, &Search_Plan_FSM::updateOdomCallback, this);
  sub_Trigger = nh.subscribe("/traj_start_trigger", 1, &Search_Plan_FSM::triggerCallback, this);
  //pub
  pub_Target = nh.advertise<geometry_msgs::PoseStamped>("/search_plan/pos_cmd", 50);
  //timer
  timer_FSM = nh.createTimer(ros::Duration(1/exec_Frequency), &Search_Plan_FSM::execFSMCallback, this);
}
//
}