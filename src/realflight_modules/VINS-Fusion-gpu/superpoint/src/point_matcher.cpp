#include "point_matcher.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>


PointMatcher::PointMatcher(const PointMatcherConfig& config) : _config(config){
  if(_config.matcher == 0){ // lightglue
    _config.dla_core = -1;
    _config.input_tensor_names.push_back("keypoints_0");
    _config.input_tensor_names.push_back("keypoints_1");
    _config.input_tensor_names.push_back("descriptors_0");
    _config.input_tensor_names.push_back("descriptors_1");
    _config.output_tensor_names.push_back("scores");

    _lightglue = std::shared_ptr<SuperPointLightGlue>(new SuperPointLightGlue(_config));
    if (!_lightglue->build()){
      std::cout << "Erron lightglue building" << std::endl;
    }
  }
  else{
    std::cout << "Plese select the point matcher! (0 for lightglue and 1 for superglue)" << std::endl;
    exit(0);
  }
}

void PointMatcher::NormalizeKeypoints(const Eigen::Matrix<float, 259, Eigen::Dynamic> &features, 
                                      Eigen::Matrix<float, 259, Eigen::Dynamic>& normalized_features, 
                                      int width, int height, float scale) {
  normalized_features = features;
  float L_inv = 1.0 / std::max(width, height) * scale;
  for (int col = 0; col < features.cols(); ++col) {
    normalized_features(1, col) = (features(1, col) - width / 2) * L_inv;
    normalized_features(2, col) = (features(2, col) - height / 2) * L_inv;
  }
}

int PointMatcher::MatchingPoints(const Eigen::Matrix<float, 259, Eigen::Dynamic>& features0,
                                  const Eigen::Matrix<float, 259, Eigen::Dynamic>& features1, 
                                  std::vector<cv::DMatch>& matches, bool outlier_rejection){
  if(features0.cols() < 1 || features1.cols() < 1){
    return 0;
  }

  Eigen::Matrix<float, 259, Eigen::Dynamic> normalized_features0, normalized_features1; 
  float scale = _config.matcher ? 0.7 : 0.5;
  NormalizeKeypoints(features0, normalized_features0, _config.image_width, _config.image_height, scale);
  NormalizeKeypoints(features1, normalized_features1, _config.image_width, _config.image_height, scale);

  matches.clear();
  std::vector<cv::Point> points0, points1;
  if(_config.matcher == 0){ // lightglue
    Eigen::Matrix<int, Eigen::Dynamic, 2> matches_index;
    Eigen::Matrix<float, Eigen::Dynamic, 1> matches_score;
    _lightglue->infer(normalized_features0.bottomRows(258), normalized_features1.bottomRows(258), matches_index, matches_score);

    for (size_t i = 0; i < matches_index.rows(); i++) {
      matches.emplace_back(matches_index(i, 0), matches_index(i, 1), 1.0 - matches_score(i));
      if(outlier_rejection){
        points0.emplace_back(features0(1, matches_index(i, 0)), features0(2, matches_index(i, 0)));
        points1.emplace_back(features1(1, matches_index(i, 1)), features1(2, matches_index(i, 1)));
      }
    }
  }
  else
  {
    std::cout << "Plese select the point matcher! (0 for lightglue and 1 for superglue)" << std::endl;
    exit(0);
  }
  // reject outliers
  if(outlier_rejection && matches.size() > 8){
    std::vector<uchar> inliers;
    cv::findFundamentalMat(points0, points1, cv::FM_RANSAC, _config.F_threshold, 0.99, inliers);
    int j = 0;
    for(int i = 0; i < matches.size(); i++){
      if(inliers[i]){
        matches[j++] = matches[i];
      }
    }
    ROS_DEBUG("remove %d points through F_RANSAC.", matches.size() - j);
    matches.resize(j);
  }

  return matches.size();
}

void PointMatcher::prewarmInference()
{
    auto dummy_feats0 = Eigen::MatrixXf::Random(259, 200);
    auto dummy_feats1 = Eigen::MatrixXf::Random(259, 200);
    std::vector<cv::DMatch> matches;
    MatchingPoints(dummy_feats0, dummy_feats1, matches, true);
    std::cout << "prewarm for lightglue completed!" << std::endl;
}