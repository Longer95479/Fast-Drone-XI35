#ifndef SEARCH_PLAN_FSM
#define SEARCH_PLAN_FSM

#include <mutex>
#include <queue>
#include <string>

#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <ros/timer.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/TakeoffLand.h>

#include <search_plan/SearchService.h>

namespace auto_search
{
    
	class Search_Plan_FSM
	{
	private:
		enum MAIN_STATE
		{
		STARTING_AREA_STAGE,
		SEARCH_STAGE,
		LAND_STAGE
		};
		enum START_SUB_STATE
		{
			TAKE_OFF,
			FLY_TO_START,
			WAIT_FOR_START
		};
		enum SEARCH_SUB_STATE
		{
			SEARCH_NUMS,
			SLOWDOWN_FOR_RECOG,
			FLY_TO_MY_TARGET,
			WAIT_FOR_LAND
		};
		enum LAND_SUB_STATE
		{
			CHECK_NUM_BE_SEEN,
			SEARCH_NEAR_NUM,
			FINE_TUNE,
			TAKE_LAND
		};
		// state
		MAIN_STATE main_State;
		START_SUB_STATE start_SubState;
		SEARCH_SUB_STATE search_SubState;
		LAND_SUB_STATE land_SubState;

		// statistic
		uint32_t continously_called_times_{0};
		
		// ros
		ros::Publisher pub_Target;
		ros::Publisher pub_Land;

		ros::Subscriber sub_Odom;
		ros::Subscriber sub_Trigger;
		ros::Subscriber sub_target_merged;

		ros::ServiceServer srv_slowdown;

		ros::Timer timer_FSM;


		// param
		std::string odom_Topic;
		double exec_Frequency, arrive_Threshold;
		// main var
		bool have_trigger  = false;
		Eigen::Vector3d current_Target;
		Eigen::Vector3d last_Target;
		Eigen::Vector3d current_Position;
		Eigen::Vector3d search_StartPoint;

		// search strategy waypoints
		double waypoints_[50][3];
		int waypoint_num_, wp_id_;
		std::vector<Eigen::Vector3d> wps_;

		bool has_odom_;
		bool has_found_my_target_;
		bool has_slow_down_req_;
		bool reset_slow_down_clock_;
		std::queue<std::pair<double, Eigen::Vector3d>> my_target_stamped_queue_;
		double target_msg_timeout_;
		double target_converge_th_;
		double slow_down_time_duration_;	// unit: sec
		double distance_to_target_;
		double slow_down_height_;
		double my_target_hover_height_;
		double publish_target_threshold_;


		// callback
		void execFSMCallback(const ros::TimerEvent &e);
		void updateOdomCallback(const nav_msgs::OdometryConstPtr &msg);
		void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
		void targetToSearchCallBack(const geometry_msgs::PoseStampedPtr &msg);
		bool slowDownServiceCallBack(search_plan::SearchService::Request  &req,
                             		 search_plan::SearchService::Response &res);
		// function
		void publishTarget();		// 发布飞行目标点,只有目标点跟上次不一样时才发布
		void publishLand();
		void execStartStage();		// 执行起始阶段状态机
		void execSearchStage();		// 执行搜索阶段状态机
		void execLandStage();		// 执行降落阶段状态机
		bool haveArrivedTarget();	// 是否到达当前目标点
		void readGivenWps();		// 初始化读取给定的一系列搜索策略目标点
		bool targetBeSeenByMyself(const ros::Time &now_time);
		void getDistanceToTarget();

		void printFSMExecState();
		void changeMainState(MAIN_STATE next_state, std::string pos_call);
		void changeStartSubState(START_SUB_STATE next_state, std::string pos_call);
		void changeSearchSubState(SEARCH_SUB_STATE next_state, std::string pos_call);
		void changeLandSubState(LAND_SUB_STATE next_state, std::string pos_call);
		void initState();
	public:
		void init(ros::NodeHandle& nh);
  	Search_Plan_FSM(/* args */)
		{

		}
    ~Search_Plan_FSM()
		{

		}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
  
}
#endif