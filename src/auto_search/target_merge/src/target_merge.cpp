#include "target_merge/target_merge.h"

namespace target_merge
{
int drone_id;
std::string PUB_TARGET_TOPIC,PUB_TARGET_SEARCH_TOPIC, SUB_TARGET_TOPIC, SUB_PNP_TOPIC, SEARCH_SERVICE_NAME;
bool open_visualization = false;
//自身目标识别回调
void Target_Merge::singleTargetCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  double time = msg->header.stamp.toSec();
  uint8_t type = msg->pose.position.z;
  if(type >= 4 || type <= 0)
    return;
  SingleTargetPtr st_p = std::make_shared<SingleTarget_Type>(time, type, msg->pose.position.x, msg->pose.position.y);
  updateSingleTarget(st_p);
}
//基于Welford更新均值和方差
void Target_Merge::updateSingleTarget(const SingleTargetPtr &target)
{
  ROS_INFO("RECEIVE Single target: %d, (%lf, %lf)", target->type, target->position.x(), target->position.y());
  int index = target->type - 1;
  auto& st = single_Merged[index];
  if(st.observed_Counts == 0)
  {//第一次观察到该目标
    st.position = target->position;
    st.cov = Matrix2d::Zero();//协方差初始化为0
    st.observed_Counts++;
  }
  else
  {
    //若点数大于20则可以进行异常值剔除
    if(st.observed_Counts >= single_merged_threshold)
    {
      double z_x = (target->position.x() - st.position.x()) / std::sqrt(st.cov(0, 0));
      double z_y = (target->position.y() - st.position.y()) / std::sqrt(st.cov(1, 1));
      if(z_x > z_core_threshold || z_y > z_core_threshold)
      {
        ROS_WARN("detect the outliers! the z_core is (%lf, %lf)", z_x, z_y);
        target->type = 0;//异常点type为0
        target_List[index].push_back(target);
        return;
      }
    }
    //更新均值和协方差
    int k = ++(st.observed_Counts);
    Vector2d temp = st.position;
    st.position += (target->position - st.position)/k;
    st.cov += (target->position - temp)*(target->position- st.position).transpose();
    target_List[index].push_back(target);
    //方差大于阈值重置统计结果
    if(st.observed_Counts <= single_merged_threshold && (st.cov(0,0) > cov_threshold_1 || st.cov(1,1) > cov_threshold_1))
    {//累积次数小于一定次数，方差不能大于较小值，否则重置
      ROS_WARN("the target-%d's cov (%lf, %lf) is exceed the threshold %lf, will reset the value", target->type, st.cov(0,0), st.cov(1,1), cov_threshold_1);
      st.reset();
      return;
    }
    else if(st.observed_Counts > single_merged_threshold && (st.cov(0,0) > cov_threshold_2 || st.cov(1,1) > cov_threshold_2))
    {//累积次数大于一定次数，方差不能大于较大值，否则重置
      ROS_WARN("the target-%d's cov (%lf, %lf) is exceed the threshold %lf, will reset the value", target->type, st.cov(0,0), st.cov(1,1), cov_threshold_2);
      st.reset();
      return;
    }
  }
  //如果还没对该目标发起过悬停且识别次数大于阈值，则请求悬停
  if(!st.have_call_slowDown && st.observed_Counts >= slow_down_counts)
  {
    if(callSearchService(Target_Merge::Slow_Down))
    {
      search_state = 1;
      search_type = target->type;
      st.have_call_slowDown = true;
      ROS_WARN("callSearchService: Slow_Down, type: %d", target->type);
    }
  }
  //如果该目标观测次数超过一定阈值就加入到融合结果
  if(st.observed_Counts % single_merged_threshold == 0)
  {
    double cur_time = ros::Time::now().toSec();
    TargetMergedPtr tm_p = std::make_shared<TargetMerged_Type>(cur_time, target->type, st.position, st.cov);
    updateTargetMerged(tm_p);
    //如果处于减速或悬停，请求正常
    if(search_state == 1 && search_type == target->type)
    {
      if(callSearchService(Target_Merge::Normal)) {
        search_state = 0;
        ROS_WARN("callSearchService: Normal, type: %d", target->type);
      }
    }
  }
}

//多机广播目标信息回调
void Target_Merge::targetMergedCallback(const target_merge::TargetMerged_MessageConstPtr &msg)
{
  TargetMergedPtr tm_p = std::make_shared<TargetMerged_Type>(msg->time, msg->type);
  tm_p->position << msg->x, msg->y;
  auto cov_array = msg->cov;
  tm_p->cov = Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>>(cov_array.data());
  ROS_INFO("Recv Target merged from %d, type:%d, position:(%lf, %lf), cov:[%lf, %lf, %lf, %lf]", msg->id, tm_p->type, tm_p->position.x(), tm_p->position.y(), 
          tm_p->cov(0,0), tm_p->cov(0,1), tm_p->cov(1,0), tm_p->cov(1,1));
  updateTargetMerged(tm_p);
}
//单机识别超过一定次数调用该函数进行融合，其他无人机融合信息接收到一次就调用进行融合
void Target_Merge::updateTargetMerged(const TargetMergedPtr &target)
{
  int index = target->type -1;
  auto& tm = target_Merged[index];
  if(tm.time == 0)
  {//尚未有融合结果
    tm = *target;
    //立即广播
    pubTargetMerged(*target);
    //发现搜寻目标，发布给search模块
    if(tm.type == drone_id)
      pubTargetToSearch(drone_id);
    //进行rviz显示
    if(open_visualization)
      targetVisualization(tm);
  }
  else if(target->time != tm.time)//不融合重复的信息
  {//kf
    auto k = tm.cov * (tm.cov + target->cov).inverse();
    tm.position += k * (target->position - tm.position);
    tm.cov = (Matrix2d::Identity() - k)*tm.cov;
    tm.time = target->time;
    //如果方差过大，重置kf
    if(tm.cov(0, 0) > kf_cov_threshold || tm.cov(1, 1) > kf_cov_threshold)
    {
      ROS_WARN("Merged target's cov (%lf, %lf) is exceed the threshold %lf, will reset the value", tm.cov(0, 0), tm.cov(1, 1), kf_cov_threshold);
      tm.reset();
      return;
    }
    //搜寻目标有更新，发布给search模块
    if(tm.type == drone_id)
      pubTargetToSearch(drone_id);
    //rviz显示
    if(open_visualization)
      targetVisualization(tm);
    ROS_INFO("Merged ok! type :%d", tm.type);
  }
}
//发布融合信息
void Target_Merge::pubTargetMerged(const TargetMerged_Type &target)
{
  target_merge::TargetMerged_Message msg;
  msg.header.stamp = ros::Time::now();
  msg.time = target.time;
  msg.id = drone_id;
  msg.type = target.type;
  msg.x = target.position.x();
  msg.y = target.position.y();
  for(int i = 0; i < 4; i++)
    msg.cov[i] = target.cov.data()[i];
  pub_TargetMerged.publish(msg);
}
//发布搜寻目标
void Target_Merge::pubTargetToSearch(int drone_id)
{
  int index = drone_id - 1;
  auto& tm = target_Merged[index];
  if(tm.time == 0)//尚未发现目标
  {
    ROS_WARN("No Search Target!");
    return;
  }
  geometry_msgs::PoseStamped msg;
  msg.header.stamp.fromSec(tm.time);
  msg.pose.position.x = tm.position.x();
  msg.pose.position.y = tm.position.y();
  msg.pose.position.z = 0.0;
  pub_TargetToSearch.publish(msg);
  ROS_WARN("Send msg to search_plan:%d", drone_id);
}
//定期广播所有融合信息
void Target_Merge::targetPubCallback(const ros::TimerEvent &e)
{
  for(int i = 0; i < 3; i++)
  {
    if(target_Merged[i].time != 0)
    {
      pubTargetMerged(target_Merged[i]);
      ROS_INFO("Current target merged: %d, position:[%lf, %lf], cov: [%lf, %lf, %lf, %lf]", target_Merged[i].type, target_Merged[i].position.x(), target_Merged[i].position.y(),
                target_Merged[i].cov(0,0), target_Merged[i].cov(0,1), target_Merged[i].cov(1,0), target_Merged[i].cov(1,1));
    }
  }
}
//向search发起服务请求
bool Target_Merge::callSearchService(SearchService_Type req_type)
{
  search_plan::SearchService srv;
  srv.request.req_type = static_cast<int>(req_type);
  return client_Search.call(srv);
}
//可视化target
void Target_Merge::targetVisualization(const TargetMerged_Type &target)
{
  visualization_msgs::Marker mark_msg;
  mark_msg.header.frame_id = "world";
  mark_msg.header.stamp = ros::Time::now();
  mark_msg.type = visualization_msgs::Marker::SPHERE;
  mark_msg.pose.position.x = target.position.x();
  mark_msg.pose.position.y = target.position.y();
  mark_msg.pose.position.z = 0.0;
  switch (target.type)
  {
  case 1:
  {
    mark_msg.color.r = 1.0;
    mark_msg.color.g = 0.0;
    mark_msg.color.b = 0.0;
    mark_msg.color.a = 1.0;
    break;
  }
  case 2:
  {
    mark_msg.color.r = 0.0;
    mark_msg.color.g = 1.0;
    mark_msg.color.b = 0.0;
    mark_msg.color.a = 1.0;
    break;
  }
  case 3:
  {
    mark_msg.color.r = 0.0;
    mark_msg.color.g = 0.0;
    mark_msg.color.b = 1.0;
    mark_msg.color.a = 1.0;
    break;
  }
  }
  mark_msg.scale.x = 0.2;
  mark_msg.scale.y = 0.2;
  mark_msg.scale.z = 0.05;
  pub_TargetRviz.publish(mark_msg);
}
void Target_Merge::init(ros::NodeHandle &nh)
{
  //ros
  nh.param<int>("/target_merge_node/single_merged_threshold", single_merged_threshold, 20);
  nh.param<int>("/target_merge_node/slow_down_counts", slow_down_counts, 3);
  nh.param<double>("/target_merge_node/z_core_threshold", z_core_threshold, 2.5);
  nh.param<double>("/target_merge_node/cov_threshold_1", cov_threshold_1, 0.05);
  nh.param<double>("/target_merge_node/cov_threshold_2", cov_threshold_2, 1.0);
  nh.param<double>("/target_merge_node/kf_cov_threshold", kf_cov_threshold, 4.0);
  nh.param<int>("/target_merge_node/drone_id", drone_id, 1);
  nh.param<double>("/target_merge_node/target_PubDuration", target_PubDuration, 2);
  nh.param<bool>("/target_merge_node/open_visualization", open_visualization, false);

  nh.param<std::string>("/target_merge_node/pub_target_merged_topic", PUB_TARGET_TOPIC, "/target_merge/pub_target_merged");
  nh.param<std::string>("/target_merge_node/pub_target_to_search_topic", PUB_TARGET_SEARCH_TOPIC, "/target_merge/target_to_search");
  nh.param<std::string>("/target_merge_node/sub_target_merged_topic", SUB_TARGET_TOPIC, "/communication/sub_target_merged");
  nh.param<std::string>("/target_merge_node/sub_pnp_topic", SUB_PNP_TOPIC, "/detect_box_pnp/target");
  nh.param<std::string>("/target_merge_node/search_service_name", SEARCH_SERVICE_NAME, "/search_plan/slowdown_for_reg");

  pub_TargetMerged = nh.advertise<target_merge::TargetMerged_Message>(PUB_TARGET_TOPIC, 10);
  pub_TargetToSearch = nh.advertise<geometry_msgs::PoseStamped>(PUB_TARGET_SEARCH_TOPIC, 10);
  pub_TargetRviz = nh.advertise<visualization_msgs::Marker>("/target_merge/target_mark", 10);
  sub_TargetSingle = nh.subscribe(SUB_PNP_TOPIC, 100, &Target_Merge::singleTargetCallback, this);
  sub_TargetMerged = nh.subscribe(SUB_TARGET_TOPIC, 100, &Target_Merge::targetMergedCallback, this);
  client_Search = nh.serviceClient<search_plan::SearchService>(SEARCH_SERVICE_NAME);
  client_Search.waitForExistence();
  ROS_INFO("Search service ok!");
  timer_PubTarget = nh.createTimer(ros::Duration(target_PubDuration), &Target_Merge::targetPubCallback, this);
}
} // namespace target_merge
