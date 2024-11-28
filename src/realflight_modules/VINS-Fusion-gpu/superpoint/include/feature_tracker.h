#ifndef FEATURE_TRACKER__H
#define FEATURE_TRACKER__H

#include <vector>
#include <queue>
#include <unordered_map>
#include <cstdio>
#include <iostream>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#include <tic_toc.h>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "feature_detector.h"
#include "point_matcher.h"

using namespace std;

class FeatureTracker
{
public:
	FeatureTracker()
	{
		n_id = 0;
		//reserve the vector
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
	}
	void readIntrinsicParameter();
	void readConfigParameter(const string &config_file, const string &model_prefix_path, const string &plugin_path="");
	vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
	vector<cv::Point2f> ptsVelocity(vector<int> &cur_ids, vector<cv::Point2f> &cur_un_pts, 
                                            unordered_map<int, cv::Point2f> &cur_id_pts, unordered_map<int, cv::Point2f> &prev_id_pts);
	void setMask();
	void addPoints();
	void track_img(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
	void track_img_use_opticalflow(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
	void readIntrinsicParameter(const vector<string> &calib_file);
	void DrawMatches(const cv::Mat& ref_image, const cv::Mat& image, 
					const vector<cv::Point2f>& ref_pts, const vector<cv::Point2f>& pts,
					const vector<int>& ref_ids, const vector<int>& ids);
	void DrawTrackCnt(const cv::Mat& image, const vector<cv::Point2f>& pts, const vector<int>& ids, const unordered_map<int, int>& id_cnt_umap);
	void DrawOpticalFlow(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap);
	bool inBorder(const cv::Point2f &pt);
	void prewarmForTracker();
	cv::Mat getTrackImage();
	void calTrackCnt();


	FeatureDetectorPtr feature_detector;
	PointMatcherPtr point_matcher;

	cv::Mat prev_img, cur_img, right_img;

	vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
	Eigen::Matrix<float, 259, Eigen::Dynamic> prev_features, cur_features, cur_right_features;

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

	cv::Mat mask;
	cv::Mat imTrack;
	long n_id;
	double cur_time, prev_time;
	bool stereo_cam;
	bool first_image_flag = true;
};

#endif