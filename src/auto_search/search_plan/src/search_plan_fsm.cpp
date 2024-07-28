#include <string>
#include "search_plan_fsm.h"

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
  switch (search_SubState)
  {
  case SEARCH_NUMS:
  {
    current_Target = wps_[wp_id_];
    // TODO: or generate wp in runtime based on search strategy

    if (has_found_my_target_) {
      changeSearchSubState(FLY_TO_MY_TAEGET);
    }
    else if (has_slow_down_req_) {
      changeSearchSubState(SLOWDOWN_FOR_RECOG);
    }
    else if (haveArrivedTarget()) {
      wp_id_++;
    }

    break;
  }

  case SLOWDOWN_FOR_RECOG:
  {
    /* TODO: stop for a while */
    break;
  }

  case FLY_TO_MY_TARGET:
  {
    current_Target = my_target_stamped_queue_.back().second;

    if (haveArrivedTarget()) {
      changeSearchSubState(WAIT_FOR_LAND);
    }

    break;
  }

  case WAIT_FOR_LAND:
  {
    /* TODO: stop for a while */

    changeMainState(LAND_STAGE);

    break;
  }
  
  default:
    break;
  }

}

//执行降落阶段状态机
void Search_Plan_FSM::execLandStage()
{
  switch (land_SubState)
  {
  case CHECK_NUM_BE_SEEN:
  {
    if (targetBeSeenByMyself(ros::Time::now())) {

      if (has_found_my_target_) {   // This if is to reduce the var copy time
        std::queue<std::pair<double, Eigen::Vector3d>> my_target_stamped_queue = my_target_stamped_queue_;

        Eigen::Vector3d target_front, target_back;
        Eigen::Vector3d target_average(0, 0, 0);

        // check error between two adjacent recieved targets' position in msg queue
        while (my_target_stamped_queue.size() >= 2) {
          target_front = my_target_stamped_queue.front().second;
          target_average = target_average + target_front;
          my_target_stamped_queue.pop();
          target_back = my_target_stamped_queue.front().second;

          // check error between two adjacent recieved targets' position
          if ((target_front - target_back).norm() > target_converge_th_) {
            has_found_my_target_ = false;
            break;
          }
          // travel throughout queue and error converged
          else if (my_target_stamped_queue.size() == 1 ) {
            target_average = target_average + my_target_stamped_queue.front().second;
            target_average = target_average / (double)(my_target_stamped_queue_.size());
            current_Target = target_average;
            changeLandSubState(FINE_TUNE);
          }
        }

      }

    }
    else {
      changeLandSubState(SEARCH_NEAR_NUM);
    }

    break;
  }

  case SEARCH_NEAR_NUM:
  {
    // TODO:
    // go circle and check target whether been seen
    // or stop for a while and back to CHECK_NUM_BE_SEEN
    break;
  }

  case FINE_TUNE:
  {
    if (haveArrivedTarget()) {
      changeLandSubState(TAKE_LAND);
    }
    break;
  }

  case TAKE_LAND:
  {
    //TODO publish land msg to land topic
    break;
  }

  default:
    break;
  }

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
  have_trigger = true;
  std::cout << "Triggered!" << std::endl;
}

void targetToSearchCallBack(const geometry_msgs::PoseStampedPtr &msg)
{
  has_found_my_target_ = true;

  std::pair<double, Eigen::Vector3d> my_target_stamped;

  my_target_stamped.first = msg.header.stamp.toSec();

  my_target_stamped.second(0) = msg.pose.position.x;
  my_target_stamped.second(1) = msg.pose.position.y;
  my_target_stamped.second(2) = msg.pose.position.z;

  if (my_target_stamped_queue_.size() > 5)
    my_target_stamped_queue_.pop();

  my_target_stamped_queue_.push(my_target);

}


void Search_Plan_FSM::publishTarget()
{
  double dis = (current_Target - last_Target).norm();
  if( dis >= 0.2)//新的目标点距离之前的目标点大于0.2m才发布新的目标点
  {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = current_Target.x();
    msg.pose.position.y = current_Target.y();
    msg.pose.position.z = current_Target.z();
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

void Search_Plan_FSM::readGivenWps()
{
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    // for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    // {
    //   visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    //   ros::Duration(0.001).sleep();
    // }
    wp_id_ = 0;
}

bool targetBeSeenByMyself(const ros::Time &now_time)
{
  return (now_time.toSec() - my_target_stamped_queue_.back().first) < target_msg_timeout_;
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

  has_found_my_target_ = false;
  has_slow_down_req_ = false;

  //param
  nh.param<std::string>("odom_topic", odom_Topic, "/vins_fusion/imu_propagate");
  nh.param<double>("exec_frequency", exec_Frequency, 100);
  nh.param<double>("arrive_threshold", arrive_Threshold, 0.3);
  nh.param<double>("target_msg_timeout", target_msg_timeout_, 2.0);
  nh.param<double>("target_converge_th", target_converge_th_, 0.05);
  
  nh.param<double>("search_startpoint_x", search_StartPoint.x(), 2);
  nh.param<double>("search_startpoint_y", search_StartPoint.y(), 0.5);
  nh.param<double>("search_startpoint_z", search_StartPoint.z(), 2);

  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++)
  {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  //sub
  sub_Odom = nh.subscribe(odom_Topic, 1, &Search_Plan_FSM::updateOdomCallback, this);
  sub_Trigger = nh.subscribe("/traj_start_trigger", 1, &Search_Plan_FSM::triggerCallback, this);
  sub_target_merged = nh.subscribe("/target_merge/target_to_search", 1, &Search_Plan_FSM::targetToSearchCallBack, this);

  //pub
  pub_Target = nh.advertise<geometry_msgs::PoseStamped>("/search_plan/pos_cmd", 50);

  //timer
  timer_FSM = nh.createTimer(ros::Duration(1/exec_Frequency), &Search_Plan_FSM::execFSMCallback, this);
}
//
}