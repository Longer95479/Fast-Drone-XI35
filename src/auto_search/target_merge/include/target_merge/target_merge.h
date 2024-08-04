#ifndef TAREGT_MERGE_H
#define TARGET_MERGE_H

#include <ros/ros.h>
#include <ros/timer.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <list>
#include <visualization_msgs/Marker.h>
#include "search_plan/SearchService.h"
#include "target_merge/TargetMerged_Message.h"

namespace target_merge
{
using Vector2d = Eigen::Vector2d;
using Matrix2d = Eigen::Matrix<double, 2, 2, Eigen::RowMajor>;
//单次识别结果
struct Single_Target_Info
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Single_Target_Info():time(0), type(0)
  {
    position << 0, 0;
  }
  Single_Target_Info(double _time, uint8_t _id, double _x, double _y):time(_time), type(_id)
  {
    position << _x, _y;
  }
  double time;
  uint8_t type;
  Vector2d position;
};
typedef struct Single_Target_Info SingleTarget_Type;
typedef std::shared_ptr<SingleTarget_Type> SingleTargetPtr;

//本机观测结果融合
struct Target_Single_Merged
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Target_Single_Merged():observed_Counts(0)
  {
    position << 0.0, 0.0;
    cov << 0.0, 0.0, 0.0, 0.0;
  }
  void reset()
  {
    have_call_slowDown = false;
    observed_Counts = 0;
    position << 0.0, 0.0;
    cov << 0.0, 0.0, 0.0, 0.0;
  }
  bool have_call_slowDown = false;//已经发起过悬停请求
  int observed_Counts;//观察次数
  Vector2d position;//统计均值
  Matrix2d cov;//统计方差
};
typedef struct Target_Single_Merged SingleMerged_Type;

//多机融合结果
struct Target_Merged
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Target_Merged():time(0), type(0)
  {
    position << 0.0, 0.0;
    cov << 0.0, 0.0, 0.0, 0.0;
  }
  Target_Merged(double _time, uint8_t _type):time(_time), type(_type)
  {
    position << 0.0, 0.0;
    cov << 0.0, 0.0, 0.0, 0.0;
  }
  Target_Merged(double _time, uint8_t _type, Vector2d _position, Matrix2d _cov):
  time(_time), type(_type), position(_position), cov(_cov){}
  void reset()
  {
    time = 0;
    type = 0;
    position << 0.0, 0.0;
    cov << 0.0, 0.0, 0.0, 0.0;
  }
  double time;
  uint8_t type;
  Vector2d position;
  Matrix2d cov;
};
typedef struct Target_Merged TargetMerged_Type;
typedef std::shared_ptr<TargetMerged_Type> TargetMergedPtr;

class Target_Merge
{
private:
  enum SearchService_Type
  {
    Normal,
    Slow_Down,
    Target_Get
  };
  //data
  std::list<SingleTargetPtr> target_List[3]; //存储历史识别信息
  SingleMerged_Type  single_Merged[3];//本机识别统计结果
  TargetMerged_Type target_Merged[3];//多机融合结果，定期广播

  //ros
  ros::Publisher pub_TargetMerged;
  ros::Publisher pub_TargetToSearch;
  ros::Publisher pub_TargetRviz;
  ros::Subscriber sub_TargetSingle;
  ros::Subscriber sub_TargetMerged;
  ros::Timer timer_PubTarget;
  ros::ServiceClient client_Search;
  //var
  int single_merged_threshold;//每识别指定次数的目标就将统计结果加入到融合结果
  int slow_down_counts;//识别超过一定次数就slow down
  double z_core_threshold, cov_threshold_1, cov_threshold_2, kf_cov_threshold;
  double target_PubDuration;//广播周期
  int search_state = 0;//search状态
  int search_type = 0;//正在进行减速识别的目标
  //function
  void updateSingleTarget(const SingleTargetPtr &target);//push单个目标信息，更新single_Merged
  void updateTargetMerged(const TargetMergedPtr &target);//push融合目标信息，更新target_Merged
  void pubTargetMerged(const TargetMerged_Type &target);
  bool callSearchService(SearchService_Type req_type);//向search发起服务请求
  void pubTargetToSearch(int drone_id);//向search模块发送自身需要降落的目标位置
  void targetVisualization(const TargetMerged_Type &target);//可视化target
  //callback
  void singleTargetCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void targetMergedCallback(const target_merge::TargetMerged_MessageConstPtr &msg);
  void targetPubCallback(const ros::TimerEvent &e);
public:
  void init(ros::NodeHandle &nh);
  Target_Merge(/* args */)
  {

  }
  ~Target_Merge()
  {

  }
};


}
#endif