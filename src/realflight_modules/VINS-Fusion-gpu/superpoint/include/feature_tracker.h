#ifndef FEATURE_TRACKER__H
#define FEATURE_TRACKER__H

#include <execinfo.h>
#include <tic_toc.h>

#include <csignal>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <unordered_map>
#include <vector>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "feature_detector.h"
#include "feature_predictor.h"
#include "point_matcher.h"

using namespace std;
using DescV = Eigen::Matrix<float, 64, 1>;

template <typename T>
using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

class FeatureTracker {
 public:
  FeatureTracker() {
    n_id = 0;
    // reserve the vector
    prev_pts.reserve(400);
    cur_pts.reserve(400);
    cur_right_pts.reserve(400);

    prev_un_pts.reserve(400);
    cur_un_pts.reserve(400);
    cur_right_pts.reserve(400);

    pts_velocity.reserve(400);
    right_pts_velocity.reserve(400);
    prev_ids.reserve(400);
    cur_ids.reserve(400);

    cur_heatmap = new float[640 * 480];
    cur_desc = new float[64 * 80 * 60];
  }
  ~FeatureTracker() {
    delete[] cur_heatmap;
    delete[] cur_desc;
  }

 public:
  void readIntrinsicParameter();
  void readIntrinsicParameter(const vector<string>& calib_file);
  void readConfigParameter(const string& config_file,
                           const string& model_prefix_path,
                           const string& plugin_path = "");
  void track_img(double _cur_time, const cv::Mat& _img,
                 const cv::Mat& _img1 = cv::Mat());
  void track_img_use_opticalflow(double _cur_time, const cv::Mat& _img,
                                 const cv::Mat& _img1 = cv::Mat());
  void prewarmForTracker();
  void DrawMatches(const cv::Mat& ref_image, const cv::Mat& image,
                   const vector<cv::Point2f>& ref_pts,
                   const vector<cv::Point2f>& pts, const vector<int>& ref_ids,
                   const vector<int>& ids);
  void DrawTrackCnt(const cv::Mat& image, const vector<cv::Point2f>& pts,
                    const vector<int>& ids,
                    const unordered_map<int, int>& id_cnt_umap);
  void DrawOpticalFlow(const cv::Mat& imLeft, const cv::Mat& imRight,
                       vector<int>& curLeftIds, vector<cv::Point2f>& curLeftPts,
                       vector<cv::Point2f>& curRightPts,
                       map<int, cv::Point2f>& prevLeftPtsMap,
                       set<int>& cur_retrack_id);
  cv::Mat getTrackImage();
  float* getCurDescMap() {
    if (cur_desc != nullptr) return cur_desc;
  }
  void etractLoopFeatsAndPub(ros::Publisher& pub_msg);

 private:
  vector<cv::Point2f> undistortedPts(vector<cv::Point2f>& pts,
                                     camodocal::CameraPtr cam);
  vector<cv::Point2f> ptsVelocity(vector<int>& cur_ids,
                                  vector<cv::Point2f>& cur_un_pts,
                                  unordered_map<int, cv::Point2f>& cur_id_pts,
                                  unordered_map<int, cv::Point2f>& prev_id_pts);
  void rejectWithF();
  void setMask();
  void addPoints();
  bool inBorder(const cv::Point2f& pt);

  void calTrackCnt();
  void printTrackCnt();

  void checkAndExtractCurFlow(const vector<cv::Point2f>& prev_pts,
                              const aligned_vector<DescV>& prev_desc,
                              const vector<cv::Point2f>& cur_pts,
                              aligned_vector<DescV>& cur_desc,
                              vector<uchar>& status);
  void retrackThroughDescMatch(const vector<cv::Point2f>& prev_pts,
                               aligned_vector<DescV> prev_desc,
                               vector<cv::Point2f>& cur_pts,
                               aligned_vector<DescV>& cur_desc,
                               vector<uchar>& status,
                               const vector<cv::Point2f>& cur_predict_pts);
  DescV extractSingleDesc(const cv::Point2f& tgt_pt);
  void extractDescriptors(const vector<cv::Point2f>& pts,
                          aligned_vector<DescV>& descs);
  void extractSquareROIPtsDesc(const cv::Point2f& ori_pt, int half_len,
                               vector<cv::Point2f>& pts,
                               aligned_vector<DescV>& descs);
  void extractROIFeatsWithNMS(const cv::Point2f& ori_pt, int half_len,
                              vector<cv::Point2f>& pts,
                              aligned_vector<DescV>& descs);
  pair<int, float> matchSingleDesc(const DescV& target_desc,
                                   aligned_vector<DescV>& descs);
  void extractKeyPoints(vector<cv::Point2f>& new_pts, float response_thresh,
                        float nms_range);
  vector<int> sort_indexes(vector<float>& data);
  vector<std::pair<int, cv::Point2f>> nms_process(
      const vector<cv::Point2f>& pts, const vector<int>& sorted_idx,
      float dist_thresh);

  vector<cv::Point2f> pts_ori_to_resized(const vector<cv::Point2f>& ori_pts);
  vector<cv::Point2f> pts_resized_to_ori(
      const vector<cv::Point2f>& resized_pts);

 public:
  FeatureDetectorPtr feature_detector;
  PointMatcherPtr point_matcher;
  FeaturePredictor feature_predictor;

  cv::Mat prev_img, cur_img, right_img;

  vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
  Eigen::Matrix<float, 259, Eigen::Dynamic> prev_features, cur_features,
      cur_right_features;  // superpoint features
  Eigen::Matrix<float, 67, Eigen::Dynamic> prev_xfeatures, cur_xfeatures,
      cur_right_xfeatures;  // xfeat features

  aligned_vector<DescV> prev_xdesc, cur_xdesc;

  vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
  vector<cv::Point2f> pts_velocity, right_pts_velocity;

  vector<int> prev_ids, cur_ids, right_ids;

  unordered_map<int, cv::Point2f> prev_un_pts_map, cur_un_pts_map;
  unordered_map<int, cv::Point2f> prev_un_right_pts_map, cur_un_right_pts_map;

  vector<int> track_cnt;
  unordered_map<int, int> prev_trackcnt_umap, cur_trackcnt_umap;

  map<int, cv::Point2f> prevLeftPtsMap;

  vector<cv::Point2f> n_pts;

  vector<camodocal::CameraPtr> m_camera;
  FeatureTrackerConfig feature_tracker_config;

  set<int> retrack_ids;

  // used for loop closure
  vector<cv::Point2f> loop_local_pts, loop_local_un_pts;
  aligned_vector<DescV> loop_local_xdesc;

  float *cur_heatmap = nullptr, *cur_desc = nullptr;

  tarckAssistArg track_assist_args;
  cv::Mat mask;
  cv::Mat imTrack;
  long n_id;
  double cur_time, prev_time;
  bool stereo_cam;
  bool first_image_flag = true;
};

#endif