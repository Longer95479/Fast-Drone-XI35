#pragma once
#include <iostream>
#include <queue>
#include <unordered_map>
#include <opencv2/features2d.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"

#include "read_configs.h"
#include "tic_toc.h"
#include "line_descriptor_custom.hpp"

#include <sensor_msgs/PointCloud.h>

using namespace cv::line_descriptor;
using namespace std;
using namespace cv;
using namespace Eigen;
using namespace camodocal;

typedef enum
{
    VERTICAL = 1,
    Hold   //到后端区再分出平行
}LineType;

class FrameLines
{
public:

    int frame_id;
    Mat img;

    vector<int> lineID;
    vector<KeyLine> keyLsd;
    Mat lbdDesc;
    
    vector<Vector4d> lineSpEpUndist;
    vector<Vector4d> lineVelocity;
    vector<LineType> lineType;
    
    vector<vector<cv::Point2f>> lineAssociaPts;

    unordered_map<int, int> trackCnt;
    unordered_map<int, Vector4d> un_id_linePts;
};
typedef shared_ptr<FrameLines> FrameLinesPtr;

class LineFeatureTracker
{
public:
    LineFeatureTracker();
    bool inBorder(const KeyLine &line);

    double getTwoLinesAbsAngle(const KeyLine &line0, const KeyLine &line1);
    double getTwoLinesAbsAngle(const Vector4d &line0, const Vector4d &line1);

    double getTwoLinesDistByP2L(const KeyLine &line0, const KeyLine &line1);
    double getTwoLinesDistByP2L(const Vector4d &line0, const Vector4d &line1);

    double getTwoLinesDistByP2P(const KeyLine &line0, const KeyLine &line1);
    double getTwoLinesDistByMid(const KeyLine &line0, const KeyLine &line1);

    void readConfigParameter(const string &config_file);
    void readIntrinsicParameter();
    void readImage(double _cur_time, const cv::Mat &_img);
    vector<int> lineNMSProcess(const vector<KeyLine> &vecTracked, const vector<KeyLine> &vecNew);
    cv::Point2f getVpzFromZc();
    vector<LineType> lineClassify(const vector<Vector4d> &key_lsd, const cv::Point2f &vp_z);
    void undistortedLineEndPoints(const vector<KeyLine> &key_lsd, vector<Vector4d> &line_undist);
    void calCurTrackCnt();
    void calCurVelocity();
    void DrawLine();
    void DrawLineWithType();
    void DrawLIneWithAssociaPts();
    void zAxisInCameraCallback(const sensor_msgs::PointCloudConstPtr &zc_msg);
    void calAssociaPtsForLines(const vector<cv::Point2f> &cur_un_pts, const vector<cv::Point2f> &cur_pts, const vector<Vector4d> &cur_un_lines, vector<vector<cv::Point2f>> &associa_pts);
    cv::Mat getTrackImage();


    FrameLinesPtr prevFrame, curFrame;

    cv::Mat undist_map1, undist_map2, K_;

    LineTrackerConfig line_tracker_config;

    CameraPtr m_camera;

    std::mutex mtx_z;
    Vector3d cur_z;
    bool is_z_usable = false;

    cv::Mat imTrack;
    long line_id;
	double cur_time, prev_time;
    bool first_image_flag = true;
};