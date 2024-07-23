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
      LANDING_STAGE
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
			WAIT_FOR_LANDING
		};
		enum LANDING_SUB_STATE
		{
			FLY_TO_NUMS,
			FINE_TUNE,
			TAKE_LAND
		};
		//state
		MAIN_STATE main_State;
		START_SUB_STATE start_SubState;
		SEARCH_SUB_STATE search_SubState;
		LANDING_SUB_STATE landing_SubState;
		
		//ros
		ros::Publisher pub_Target;
		ros::Subscriber sub_Odom;
		ros::Timer timer_FSM;
		//main var
		Eigen::Vector3d current_Target;
		Eigen::Vector3d last_Target;
		Eigen::Vector3d current_Position;

		//callback
		void execFSMCallback(const ros::TimerEvent &e);
		void updateOdomCallback(const nav_msgs::OdometryConstPtr &msg);
		//function
		void publishTarget();//发布飞行目标点,只有目标点跟上次不一样时才发布
		void execStartStage();//执行起始阶段状态机
		void execSearchStage();//执行搜索阶段状态机
		void execLandStage();//执行降落阶段状态机
	public:
		void init(ros::NodeHandle& nh);
  	Search_Plan_FSM(/* args */)
		{

		}
    ~Search_Plan_FSM()
		{

		}
	};
  
}
#endif