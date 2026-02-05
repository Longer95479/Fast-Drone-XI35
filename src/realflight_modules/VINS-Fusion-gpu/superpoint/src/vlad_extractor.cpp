#include "vlad_extractor.h"

#include <algorithm>

#include "superpoint/GlobalFeature.h"

void VLADExtractor::init(ros::NodeHandle& nh, std::string config_file,
                         std::string model_prefix_path) {
  pub_vlad = nh.advertise<superpoint::GlobalFeature>(
      "/feature_tracker/global_feature", 1000);
  NetVLADConfig netvlad_config;
  netvlad_config.load(config_file);
  netvlad_config.setModelPrefixPath(model_prefix_path);
  netvlad_ = std::make_shared<NetVLAD>(netvlad_config);
  if (!netvlad_->build()) {
    std::cout << "Error occured when building netvlad engine." << std::endl;
  }
}

void VLADExtractor::moveFeatsIn(double timestamp, float** feats_desc,
                                int feats_num) {
  if (cur_feats_desc != nullptr) {
    free(cur_feats_desc);
  }
  cur_feats_desc = *feats_desc;
  *feats_desc = nullptr;
  cur_feats_num = feats_num;
  cur_time = timestamp;
}

void VLADExtractor::copyFeatsIn(double timestamp, const float* feats_desc,
                                int feats_num, int feats_dim) {
  int n_bytes = feats_num * feats_dim * sizeof(float);
  if (cur_feats_desc == nullptr) {
    cur_feats_desc = (float*)malloc(n_bytes);
  }
  memcpy(cur_feats_desc, feats_desc, n_bytes);
  cur_feats_num = feats_num;
  cur_time = timestamp;
}

bool VLADExtractor::extractVLAD(const float* local_feats_desc,
                                int local_feats_num,
                                Eigen::VectorXf& vlad_feat) {
  bool good_infer =
      netvlad_->infer(local_feats_desc, local_feats_num, vlad_feat);
  if (!good_infer) {
    ROS_WARN("Failed when extracting vlad feature.");
  }
  return good_infer;
}

bool VLADExtractor::extractVLAD(Eigen::VectorXf& vlad_feat) {
  if (cur_feats_desc == nullptr) return false;
  bool good_infer = netvlad_->infer(cur_feats_desc, cur_feats_num, vlad_feat);
  if (!good_infer) {
    ROS_WARN("Failed when extracting vlad feature.");
  }
  return good_infer;
}

void VLADExtractor::processAndPublish() {
  Eigen::VectorXf cur_vlad;
  if (extractVLAD(cur_vlad)) {
    superpoint::GlobalFeature msg;
    msg.stamp = ros::Time(cur_time);
    for (size_t i = 0; i < cur_vlad.size(); i++) {
      msg.descriptor[i] = cur_vlad[i];
    }
    pub_vlad.publish(msg);
  }
}

void VLADExtractor::prewarm() {
  int feats_dim = 64, feats_num = 4800;
  float* dummy_feats = new float[feats_dim * feats_num];
  std::fill_n(dummy_feats, feats_dim * feats_num, 0.5);
  Eigen::VectorXf dummy_vlad;
  if (extractVLAD(dummy_feats, feats_num, dummy_vlad)) {
    std::cout << "Prewarm for vlad extractor completed!" << std::endl;
  } else {
    std::cout << "Failed to prewarm vlad extarctor!" << std::endl;
  }
  delete[] dummy_feats;
}