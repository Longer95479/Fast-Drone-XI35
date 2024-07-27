#ifndef SEARCH_PLAN_FSM
#define SEARCH_PLAN_FSM
#include <mutex>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <ros/timer.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

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
			WAIT_FOR_LAND
		};
		enum LAND_SUB_STATE
		{
			FLY_TO_NUMS,
			FINE_TUNE,
			TAKE_LAND
		};
		//state
		MAIN_STATE main_State;
		START_SUB_STATE start_SubState;
		SEARCH_SUB_STATE search_SubState;
		LAND_SUB_STATE land_SubState;
		
		//ros
		ros::Publisher pub_Target;
		ros::Subscriber sub_Odom;
		ros::Subscriber sub_Trigger;
		ros::Timer timer_FSM;
		//param
		std::string odom_Topic;
		double exec_Frequency, arrive_Threshold;
		//main var
		bool have_trigger  = false;
		Eigen::Vector3d current_Target;
		Eigen::Vector3d last_Target;
		Eigen::Vector3d current_Position;
		Eigen::Vector3d search_StartPoint;


		//callback
		void execFSMCallback(const ros::TimerEvent &e);
		void updateOdomCallback(const nav_msgs::OdometryConstPtr &msg);
		void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
		//function
		void publishTarget();//发布飞行目标点,只有目标点跟上次不一样时才发布
		void execStartStage();//执行起始阶段状态机
		void execSearchStage();//执行搜索阶段状态机
		void execLandStage();//执行降落阶段状态机
		bool haveArrivedTarget();//是否到达当前目标点
		void changeMainState(MAIN_STATE next_state);
		void changeStartSubState(START_SUB_STATE next_state);
		void changeSearchSubState(SEARCH_SUB_STATE next_state);
		void changeLandSubState(LAND_SUB_STATE next_state);
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