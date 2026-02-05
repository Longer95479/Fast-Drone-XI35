#pragma once
#include <ros/ros.h>

#include "netvlad.h"

class VLADExtractor {
 public:
  void init(ros::NodeHandle& nh, std::string config_file,
            std::string model_prefix_path);
  void moveFeatsIn(double timestamp, float** feats_desc, int feats_num);
  void copyFeatsIn(double timestamp, const float* feats_desc, int feats_num,
                   int feats_dim = 64);
  bool extractVLAD(const float* local_feats_desc, int local_feats_num,
                   Eigen::VectorXf& vlad_feat);
  bool extractVLAD(Eigen::VectorXf& vlad_feat);
  void processAndPublish();
  void prewarm();

 private:
  ros::Publisher pub_vlad;

  float* cur_feats_desc;
  int cur_feats_num;
  double cur_time;
  NetVLADPtr netvlad_;
};
typedef std::shared_ptr<VLADExtractor> VLADExtractorPtr;