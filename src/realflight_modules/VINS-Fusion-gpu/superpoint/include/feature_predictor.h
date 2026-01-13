#pragma once

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <deque>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class FeaturePredictor {
  public:
    FeaturePredictor();
    void init(ros::NodeHandle& nh);
    void loadExtrinsic(const std::string& config_file);
    void setCameraParam(const camodocal::CameraPtr& camera_ptr, int img_width, int img_height);
    void resetReferenceFeature(double ts, const std::vector<cv::Point2f>& cur_pts);
    void resetReferenceFeature(
        double ts, const std::vector<cv::Point2f>& cur_pts,
        const std::vector<cv::Point2f>& cur_un_pts);
    void getPrediction(double cur_frame_ts, std::vector<cv::Point2f>& cur_pts);
    void update(const std::vector<cv::Point2f>& cur_pts, const std::vector<uchar>& status);

  protected:
    bool calcuReleRot(double cur_frame_ts);
    void inputImuMeas(const sensor_msgs::ImuConstPtr& imu_msg);
    void updateExtrinsicAndBias(const nav_msgs::OdometryPtr& msg);

  private:
    inline void clamp_point2f(cv::Point2f& p2f) const {
        if (p2f.x > max_x_) {
            p2f.x = max_x_;
        } else if (p2f.x < 0) {
            p2f.x = 0;
        }

        if (p2f.y > max_y_) {
            p2f.y = max_y_;
        } else if (p2f.y < 0) {
            p2f.y = 0;
        }
    }

  protected:
    struct CellFlow {
        cv::Point2f flow_sum = cv::Point2f(0, 0);
        cv::Point2f flow_avg = cv::Point2f(0, 0);
        int cnt              = 0;
    };

    static constexpr int kGridRows = 3;
    static constexpr int kGridCols = 3;
    int cell_width_                = 1;
    int cell_height_               = 1;
    double max_x_, max_y_;

    ros::Subscriber sub_imu_;
    ros::Subscriber sub_extrinsic_;

    camodocal::CameraPtr camera_;

    std::vector<cv::Point2f> ref_pts_;
    std::vector<Eigen::Vector3d> ref_un_pts_;
    std::vector<int> pts_idx_to_cell_idx_;
    std::vector<CellFlow> grid_flow_corr_;

    cv::Point2f flow_corr_avg_{0, 0};

    double ref_frame_ts_, cur_frame_ts_;

    std::mutex mtx_ext_;
    Eigen::Vector3d bias_gyro_;
    Eigen::Quaterniond q_ic_;

    Eigen::Matrix3d R_cur_ref_;

    std::mutex mtx_imu_;
    std::deque<sensor_msgs::ImuConstPtr> imu_que_;
};