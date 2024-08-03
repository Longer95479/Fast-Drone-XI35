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
      changeStartSubState(FLY_TO_START, "execStartStage()");
      // ROS_INFO("Start substate switch: TAKEOFF ======> FLY_TO_START");
    }
    break;
  }
  case FLY_TO_START:
  {
    current_Target = search_StartPoint;
    if(haveArrivedTarget())
    {
      changeStartSubState(WAIT_FOR_START, "execStartStage()");
      // ROS_INFO("Start substate switch: FLY_TO_START ======> WAIT_FOR_START");
    }
    break;
  }
  case WAIT_FOR_START:
  {
    //todo 所有无人机就绪后再进入搜索阶段
    changeMainState(SEARCH_STAGE, "execStartStage()");
    // ROS_INFO("Main state switch: STARTING_AREA_STAGE ======> SEARCH_STAGE");
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
    // TODO: or generate wp in runtime based on search strategy

    if (has_found_my_target_) {
      changeSearchSubState(FLY_TO_MY_TARGET, "execSearchStage()");
    }
    else if (has_slow_down_req_) {
      changeSearchSubState(SLOWDOWN_FOR_RECOG, "execSearchStage()");
    }
    else if ((wp_id_ < waypoint_num_ - 1) ) {
      current_Target = wps_[wp_id_];
      // note: current_Target = wps_[wp_id_] must be done before calling haveArrivedTarget().
      if (haveArrivedTarget()) {
        wp_id_++;
        current_Target = wps_[wp_id_];
      }
    }
    else
      current_Target = wps_[wp_id_];

    break;
  }

  case SLOWDOWN_FOR_RECOG:
  {
    /* TODO: stop for a while */
    static bool start_the_clock = true;
    static ros::Time start_time;
    static Eigen::Vector3d slow_down_trig_pos;
    
    if (start_the_clock) {
      start_the_clock = false;
      start_time = ros::Time::now();

      if (continously_called_times_ == 1) {
        slow_down_trig_pos = current_Position;
        slow_down_trig_pos.z() = slow_down_height_;
      }
    }
    else {
      // Now just stopping
      // TODO: slowing down instead of stopping
      current_Target = slow_down_trig_pos;

      ros::Time now_time = ros::Time::now();
      if (!has_slow_down_req_) {
        start_the_clock = true;
        has_slow_down_req_ = false;

        changeSearchSubState(SEARCH_NUMS, "execSearchStage()");

        ROS_WARN("[SLOW DOWN] recieve slow down quit req.");
      }
      else if ((now_time - start_time).toSec() > slow_down_time_duration_) {
        start_the_clock = true;
        has_slow_down_req_ = false;

        changeSearchSubState(SEARCH_NUMS, "execSearchStage()");

        ROS_WARN("[SLOW DOWN] slow down time out.");
      }
      else if (reset_slow_down_clock_ == true) {
        start_the_clock = true;
        reset_slow_down_clock_ = false;
        ROS_WARN("[SLOW DOWN] reset the slow down clock.");
      }
    }

    break;
  }

  case FLY_TO_MY_TARGET:
  {
    current_Target = my_target_stamped_queue_.back().second;
    
    //TODO: need to modify to get better land
    // current_Target.z() = current_Position.z();
    current_Target.z() = my_target_hover_height_;

    if (haveArrivedTarget()) {
      changeSearchSubState(WAIT_FOR_LAND, "execSearchStage()");
    }

    break;
  }

  case WAIT_FOR_LAND:
  {
    /* TODO: stop for a while */

    changeMainState(LAND_STAGE, "execSearchStage()");

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
          // has traveled throughout queue and error converged
          else if (my_target_stamped_queue.size() == 1 ) {
            target_average = target_average + my_target_stamped_queue.front().second;
            target_average = target_average / (double)(my_target_stamped_queue_.size());
            current_Target = target_average;

            //TODO: need to modify to get better land
            current_Target.z() = current_Position.z();
            
            changeLandSubState(FINE_TUNE, "execLandStage()");
          }
        }

      }

    }
    else {
      changeLandSubState(SEARCH_NEAR_NUM, "execLandStage()");
    }

    break;
  }

  case SEARCH_NEAR_NUM:
  {
    // TODO:
    // go circle and check target whether been seen
    // or stop for a while and back to CHECK_NUM_BE_SEEN
    // or just land without fine tune.
    changeLandSubState(TAKE_LAND, "execLandStage()");
    break;
  }

  case FINE_TUNE:
  {
    if (haveArrivedTarget()) {
      changeLandSubState(TAKE_LAND, "execLandStage()");
    }
    break;
  }

  case TAKE_LAND:
  {
    if (continously_called_times_ == 1)
      publishLand();

    static int count = 0;
    if (count < 100) {
      count++;
    }
    else {
      publishLand();
      count = 0;
    }

    break;
  }

  default:
    break;
  }

}

//状态机执行函数
void Search_Plan_FSM::execFSMCallback(const ros::TimerEvent &e)
{

  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100)
  {
    printFSMExecState();

    if (!has_odom_)
      std::cout << "no odom." << std::endl;
    else {
      std::cout << "= = = = = = = = = = = =" << std::endl;

      std::cout << "current_Position" << std::endl;
      std::cout << current_Position << std::endl;

      std::cout << "* * * *" << std::endl;

      std::cout << "current_Target" << std::endl;
      std::cout << current_Target << std::endl;

      getDistanceToTarget();
      std::cout << "dist_to_target: " << distance_to_target_ << std::endl;  
    }
    
    fsm_num = 0;
  }

  continously_called_times_++;

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
  has_odom_ = true;
}


void Search_Plan_FSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
{
  have_trigger = true;
  std::cout << "Triggered!" << std::endl;
}

void Search_Plan_FSM::targetToSearchCallBack(const geometry_msgs::PoseStampedPtr &msg)
{
  has_found_my_target_ = true;

  std::pair<double, Eigen::Vector3d> my_target_stamped;

  my_target_stamped.first = msg->header.stamp.toSec();

  my_target_stamped.second(0) = msg->pose.position.x;
  my_target_stamped.second(1) = msg->pose.position.y;
  my_target_stamped.second(2) = msg->pose.position.z;

  if (my_target_stamped_queue_.size() > 3)
    my_target_stamped_queue_.pop();

  my_target_stamped_queue_.push(my_target_stamped);

}


bool Search_Plan_FSM::slowDownServiceCallBack(search_plan::SearchService::Request  &req,
                                              search_plan::SearchService::Response &res)
{
  ROS_WARN("recieved req, %d", req.req_type);

  if (req.req_type == 0)
    has_slow_down_req_ = false;
  else if (req.req_type == 1) {
    has_slow_down_req_ = true;
    if (main_State == SEARCH_STAGE && search_SubState == SLOWDOWN_FOR_RECOG)
      reset_slow_down_clock_ = true;
  }
  else {
    ROS_ERROR("slow_down_req_ is wrong");
    return false;
  }

  res.success = true;
  return true;
}


void Search_Plan_FSM::publishTarget()
{
  // static int normal_pub_counter = 0;

  if (have_trigger) {
    double dis = (current_Target - last_Target).norm();
    if( dis >= publish_target_threshold_) //新的目标点距离之前的目标点大于 publish_target_threshold_ [m] 才发布新的目标点
    {
      geometry_msgs::PoseStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.pose.position.x = current_Target.x();
      msg.pose.position.y = current_Target.y();
      msg.pose.position.z = current_Target.z();
      pub_Target.publish(msg);
      ROS_WARN("publish new target.");

      last_Target = current_Target;
    }
  }
}


void Search_Plan_FSM::publishLand()
{
  quadrotor_msgs::TakeoffLand msg;
  msg.takeoff_land_cmd = 2;
  pub_Land.publish(msg);
}

// todo for substate
// std::pair<int, Search_Plan_FSM::FSM_EXEC_STATE> Search_Plan_FSM::timesOfConsecutiveStateCalls()
// {
//   return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
// }


// todo
void Search_Plan_FSM::printFSMExecState()
{
  static std::string main_state_str[3] = {"STARTING_AREA_STAGE", "SEARCH_STAGE", "LAND_STAGE"};
  static std::string start_substate_str[3] = {"TAKE_OFF", "FLY_TO_START", "WAIT_FOR_START"};
  static std::string search_substate_str[4] = {"SEARCH_NUMS", "SLOWDOWN_FOR_RECOG", "FLY_TO_MY_TARGET", "WAIT_FOR_LAND"};
  static std::string land_substate_str[4] = {"CHECK_NUM_BE_SEEN", "SEARCH_NEAR_NUM", "FINE_TUNE", "TAKE_LAND"};

  std::cout << "[Search FSM]: main state: " + main_state_str[int(main_State)] << std::endl;

  switch (main_State)
  {
  case STARTING_AREA_STAGE:
  {
    std::cout << "               sub state: " + start_substate_str[int(start_SubState)] << std::endl;
    break;
  }

  case SEARCH_STAGE:
  {
    std::cout << "               sub state: " + search_substate_str[int(search_SubState)] << std::endl;
    break;
  }

  case LAND_STAGE:
  {
    std::cout << "               sub state: " + land_substate_str[int(land_SubState)] << std::endl;
    break;
  }
  
  default:
    break;
  }
}


void Search_Plan_FSM::changeMainState(MAIN_STATE next_state, std::string pos_call)
{
  if (next_state != main_State)
    continously_called_times_ = 0;

  int pre_state = int(main_State);
  main_State = next_state;

  static std::string state_str[3] = {"STARTING_AREA_STAGE", "SEARCH_STAGE", "LAND_STAGE"};
  std::cout << "[" + pos_call + "]: from " + state_str[pre_state] + " to " + state_str[int(next_state)] << std::endl;

}

void Search_Plan_FSM::changeStartSubState(START_SUB_STATE next_state, std::string pos_call)
{
  if (next_state != start_SubState)
    continously_called_times_ = 0;

  int pre_state = int(start_SubState);
  start_SubState = next_state;

  static std::string state_str[3] = {"TAKE_OFF", "FLY_TO_START", "WAIT_FOR_START"};
  std::cout << "[" + pos_call + "]: from " + state_str[pre_state] + " to " + state_str[int(next_state)] << std::endl;
}

void Search_Plan_FSM::changeSearchSubState(SEARCH_SUB_STATE next_state, std::string pos_call)
{
  if (next_state != search_SubState)
    continously_called_times_ = 0;

  int pre_state = int(search_SubState);
  search_SubState = next_state;

  static std::string state_str[4] = {"SEARCH_NUMS", "SLOWDOWN_FOR_RECOG", "FLY_TO_MY_TARGET", "WAIT_FOR_LAND"};
  std::cout << "[" + pos_call + "]: from " + state_str[pre_state] + " to " + state_str[int(next_state)] << std::endl;
}

void Search_Plan_FSM::changeLandSubState(LAND_SUB_STATE next_state, std::string pos_call)
{
  if (next_state != land_SubState)
    continously_called_times_ = 0;

  int pre_state = int(land_SubState);
  land_SubState = next_state;

  static std::string state_str[4] = {"CHECK_NUM_BE_SEEN", "SEARCH_NEAR_NUM", "FINE_TUNE", "TAKE_LAND"};
  std::cout << "[" + pos_call + "]: from " + state_str[pre_state] + " to " + state_str[int(next_state)] << std::endl;
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

bool Search_Plan_FSM::targetBeSeenByMyself(const ros::Time &now_time)
{
  return (now_time.toSec() - my_target_stamped_queue_.back().first) < target_msg_timeout_;
}

void Search_Plan_FSM::getDistanceToTarget()
{
  distance_to_target_ = (current_Target - current_Position).norm();
}

//初始化各状态
void Search_Plan_FSM::initState()
{
  main_State = STARTING_AREA_STAGE;
  start_SubState = TAKE_OFF;
  search_SubState = SEARCH_NUMS;
  land_SubState = CHECK_NUM_BE_SEEN;
  ROS_INFO("Initail State: STARTING_AREA_STAGE<TAKE_OFF>");
}


void Search_Plan_FSM::init(ros::NodeHandle& nh)
{
  // var init
  initState();
  current_Target << 0.0, 0.0, 0.0;
  last_Target << 0.0, 0.0, 0.0;

  has_odom_ = false;
  has_found_my_target_ = false;
  has_slow_down_req_ = false;
  reset_slow_down_clock_ = false;

  // param
  nh.param<std::string>("/search_plan_node/odom_topic", odom_Topic, "/vins_fusion/imu_propagate");
  nh.param<double>("/search_plan_node/exec_frequency", exec_Frequency, 100);
  nh.param<double>("/search_plan_node/arrive_threshold", arrive_Threshold, 0.3);
  nh.param<double>("/search_plan_node/publish_target_threshold", publish_target_threshold_, 0.2);
  nh.param<double>("/search_plan_node/target_msg_timeout", target_msg_timeout_, 2.0);
  nh.param<double>("/search_plan_node/target_converge_th", target_converge_th_, 0.10);
  nh.param<double>("/search_plan_node/slow_down_time_duration", slow_down_time_duration_, 2.0);
  nh.param<double>("/search_plan_node/slow_down_height", slow_down_height_, 0.7);
  nh.param<double>("/search_plan_node/my_target_hover_height", my_target_hover_height_, 0.7);

  nh.param<double>("/search_plan_node/search_startpoint_x", search_StartPoint.x(), 2.0);
  nh.param<double>("/search_plan_node/search_startpoint_y", search_StartPoint.y(), 0.5);
  nh.param<double>("/search_plan_node/search_startpoint_z", search_StartPoint.z(), 2.0);

  nh.param("/search_plan_node/fsm/waypoint_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++)
  {
    nh.param("/search_plan_node/fsm/waypoint" + std::to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("/search_plan_node/fsm/waypoint" + std::to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("/search_plan_node/fsm/waypoint" + std::to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  readGivenWps();

  // sub
  sub_Odom = nh.subscribe(odom_Topic, 1, &Search_Plan_FSM::updateOdomCallback, this);
  sub_Trigger = nh.subscribe("/traj_start_trigger", 1, &Search_Plan_FSM::triggerCallback, this);
  sub_target_merged = nh.subscribe("/target_merge/target_to_search", 1, &Search_Plan_FSM::targetToSearchCallBack, this);

  // pub
  pub_Target = nh.advertise<geometry_msgs::PoseStamped>("/search_plan/pos_cmd", 50);
  pub_Land = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 5, true);

  // srv
  srv_slowdown = nh.advertiseService("/search_plan/slowdown_for_reg", &Search_Plan_FSM::slowDownServiceCallBack, this);

  // timer
  timer_FSM = nh.createTimer(ros::Duration(1/exec_Frequency), &Search_Plan_FSM::execFSMCallback, this);
}
//
}