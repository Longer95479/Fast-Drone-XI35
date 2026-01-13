#include "feature_predictor.h"
#include <chrono>
#include <opencv2/core/eigen.hpp>
#include <thread>

FeaturePredictor::FeaturePredictor() {
    bias_gyro_.setZero();
    q_ic_.setIdentity();
    R_cur_ref_.setIdentity();
}

void FeaturePredictor::init(ros::NodeHandle& nh) {
    sub_imu_       = nh.subscribe("/imu_filter/data", 100, &FeaturePredictor::inputImuMeas, this);
    sub_extrinsic_ = nh.subscribe(
        "/vins_fusion/extrinsic", 100, &FeaturePredictor::updateExtrinsicAndBias, this);
}

void FeaturePredictor::loadExtrinsic(const std::string& config_file) {
    FILE* fh = fopen(config_file.c_str(), "r");
    if (fh == NULL) {
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;
    }
    fclose(fh);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    cv::Mat cv_T;
    fsSettings["body_T_cam0"] >> cv_T;
    Eigen::Matrix4d T;
    cv::cv2eigen(cv_T, T);
    auto R_ic = T.block<3, 3>(0, 0);
    q_ic_     = Eigen::Quaterniond(R_ic);
}

void FeaturePredictor::setCameraParam(
    const camodocal::CameraPtr& camera_ptr, int img_width, int img_height) {
    if (camera_ptr != nullptr) camera_ = camera_ptr;
    cell_width_  = std::ceil((img_width + kGridCols - 1.0f) / kGridCols);
    cell_height_ = std::ceil((img_height + kGridRows - 1.0f) / kGridRows);
    grid_flow_corr_.resize(
        kGridRows * kGridCols, CellFlow{cv::Point2d{0, 0}, cv::Point2f{0, 0}, 0});
    max_x_ = img_width - 1;
    max_y_ = img_height - 1;
}

void FeaturePredictor::resetReferenceFeature(double ts, const std::vector<cv::Point2f>& cur_pts) {
    if (!ref_pts_.empty()) {
        ref_pts_.clear();
        ref_un_pts_.clear();
        pts_idx_to_cell_idx_.clear();
    }

    ref_pts_ = cur_pts;
    ref_un_pts_.resize(ref_pts_.size());
    pts_idx_to_cell_idx_.resize(ref_pts_.size());
    for (size_t i = 0; i < cur_pts.size(); i++) {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        camera_->liftProjective(a, ref_un_pts_[i]);

        pts_idx_to_cell_idx_[i] = static_cast<int>(cur_pts[i].y / cell_height_) * kGridCols +
                                  static_cast<int>(cur_pts[i].x / cell_width_);
    }
    ref_frame_ts_ = ts;
}
void FeaturePredictor::resetReferenceFeature(
    double ts, const std::vector<cv::Point2f>& cur_pts,
    const std::vector<cv::Point2f>& cur_un_pts) {
    if (!ref_pts_.empty()) {
        ref_pts_.clear();
        ref_un_pts_.clear();
        pts_idx_to_cell_idx_.clear();
    }

    ref_pts_ = cur_pts;
    ref_un_pts_.resize(ref_pts_.size());
    pts_idx_to_cell_idx_.resize(ref_pts_.size());
    for (size_t i = 0; i < cur_pts.size(); i++) {
        ref_un_pts_[i]          = Eigen::Vector3d(cur_un_pts[i].x, cur_un_pts[i].y, 1);
        pts_idx_to_cell_idx_[i] = static_cast<int>(cur_pts[i].y / cell_height_) * kGridCols +
                                  static_cast<int>(cur_pts[i].x / cell_width_);
    }
    ref_frame_ts_ = ts;
}

bool FeaturePredictor::calcuReleRot(double cur_frame_ts) {
    cur_frame_ts_ = cur_frame_ts;
    R_cur_ref_.setIdentity();
    if (imu_que_.empty() || imu_que_.front()->header.stamp.toSec() > ref_frame_ts_ + 0.5)
        return false;
    while (imu_que_.back()->header.stamp.toSec() < cur_frame_ts_) {
        ROS_WARN("FeaturePredictor: wait for imu...");
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
    // collect imu between ref_frame_ts and cur_frame_ts
    mtx_ext_.lock();
    Eigen::Vector3d cur_bias_gyro = bias_gyro_;
    Eigen::Quaterniond cur_q_ic   = q_ic_;
    mtx_ext_.unlock();
    std::vector<std::pair<double, Eigen::Vector3d>> integ_que;
    mtx_imu_.lock();
    sensor_msgs::ImuConstPtr ref_left_imu = imu_que_.front();
    while (!imu_que_.empty() && imu_que_.front()->header.stamp.toSec() < ref_frame_ts_) {
        ref_left_imu = imu_que_.front();
        imu_que_.pop_front();
    }
    if (imu_que_.front()->header.stamp.toSec() > ref_frame_ts_) {
        double ref_left_time = ref_left_imu->header.stamp.toSec();
        Eigen::Vector3d ref_left_gyro(
            ref_left_imu->angular_velocity.x, ref_left_imu->angular_velocity.y,
            ref_left_imu->angular_velocity.z);
        double ref_right_time = imu_que_.front()->header.stamp.toSec();
        Eigen::Vector3d ref_right_gyro(
            imu_que_.front()->angular_velocity.x, imu_que_.front()->angular_velocity.y,
            imu_que_.front()->angular_velocity.z);
        double coffe_1 = (ref_right_time - ref_frame_ts_) / (ref_right_time - ref_left_time);
        double coffe_2 = (ref_frame_ts_ - ref_left_time) / (ref_right_time - ref_left_time);
        Eigen::Vector3d mid_gyro = coffe_1 * ref_left_gyro + coffe_2 * ref_right_gyro;
        integ_que.emplace_back(ref_frame_ts_, (mid_gyro - cur_bias_gyro));
    }

    sensor_msgs::ImuConstPtr cur_left_imu = imu_que_.front();
    while (!imu_que_.empty() && imu_que_.front()->header.stamp.toSec() < cur_frame_ts_) {
        Eigen::Vector3d gyro_temp(
            imu_que_.front()->angular_velocity.x, imu_que_.front()->angular_velocity.y,
            imu_que_.front()->angular_velocity.z);

        integ_que.emplace_back(imu_que_.front()->header.stamp.toSec(), gyro_temp);
        integ_que.back().second -= cur_bias_gyro;
        cur_left_imu = imu_que_.front();
        imu_que_.pop_front();
    }
    double cur_left_time = cur_left_imu->header.stamp.toSec();
    Eigen::Vector3d cur_left_gyro(
        cur_left_imu->angular_velocity.x, cur_left_imu->angular_velocity.y,
        cur_left_imu->angular_velocity.z);
    double cur_right_time = imu_que_.front()->header.stamp.toSec();
    Eigen::Vector3d cur_right_gyro(
        imu_que_.front()->angular_velocity.x, imu_que_.front()->angular_velocity.y,
        imu_que_.front()->angular_velocity.z);
    double coffe_1           = (cur_right_time - cur_frame_ts_) / (cur_right_time - cur_left_time);
    double coffe_2           = (cur_frame_ts_ - cur_left_time) / (cur_right_time - cur_left_time);
    Eigen::Vector3d mid_gyro = coffe_1 * cur_left_gyro + coffe_2 * cur_right_gyro;
    integ_que.emplace_back(cur_frame_ts_, (mid_gyro - cur_bias_gyro));
    imu_que_.push_front(cur_left_imu);
    mtx_imu_.unlock();
    if (integ_que.empty()) return false;

    Eigen::Quaterniond q_i_ref_cur;
    q_i_ref_cur.setIdentity();
    auto last_gyro = integ_que[0];
    for (size_t i = 1; i < integ_que.size(); i++) {
        double dt                = integ_que[i].first - last_gyro.first;
        Eigen::Vector3d mid_gyro = 0.5 * (last_gyro.second + integ_que[i].second);
        q_i_ref_cur *= Eigen::Quaterniond(
            1, mid_gyro(0) * dt / 2., mid_gyro(1) * dt / 2., mid_gyro(2) * dt / 2.);
        last_gyro = integ_que[i];
    }
    Eigen::Quaterniond q_c_ref_cur = cur_q_ic.inverse() * q_i_ref_cur * cur_q_ic;
    R_cur_ref_                     = q_c_ref_cur.inverse().toRotationMatrix();
    return true;
}

void FeaturePredictor::getPrediction(double cur_frame_ts, std::vector<cv::Point2f>& cur_pts) {
    cur_pts.clear();
    cur_pts.resize(ref_pts_.size());
    calcuReleRot(cur_frame_ts);
    for (size_t i = 0; i < ref_pts_.size(); i++) {
        Eigen::Vector3d cur_pts_rot = R_cur_ref_ * ref_un_pts_[i];
        cv::Point2f cur_pts_corr    = grid_flow_corr_[pts_idx_to_cell_idx_[i]].flow_avg;
        if (cur_pts_rot[2] < 1e-5) {
            cur_pts[i] = ref_pts_[i] + cur_pts_corr;
        } else {
            Eigen::Vector2d cur_pts_rot_plane;
            camera_->spaceToPlane(cur_pts_rot, cur_pts_rot_plane);
            cv::Point2f cur_pts_pred =
                cv::Point2f(cur_pts_rot_plane(0), cur_pts_rot_plane(1)) + cur_pts_corr;
            clamp_point2f(cur_pts_pred);
            cur_pts[i] = cur_pts_pred;
        }
    }
}

void FeaturePredictor::update(
    const std::vector<cv::Point2f>& cur_pts, const std::vector<uchar>& status) {
    assert(ref_pts_.size() == cur_pts.size());
    for (auto& cell : grid_flow_corr_) {
        cell.flow_sum = cv::Point2f(0, 0);
        cell.flow_avg = cv::Point2f(0, 0);
        cell.cnt      = 0;
    }
    int all_flow_cnt = 0;
    flow_corr_avg_   = cv::Point2f(0, 0);
    for (size_t i = 0; i < cur_pts.size(); i++) {
        if (!status[i]) {
            continue;
        }
        Eigen::Vector3d ref_pts_rot = R_cur_ref_ * ref_un_pts_[i];
        if (ref_pts_rot[2] < 1e-5) {
            continue;
        }
        Eigen::Vector2d ref_pts_rot_plane;
        camera_->spaceToPlane(ref_pts_rot, ref_pts_rot_plane);
        cv::Point2f cur_flow_corr =
            cur_pts[i] - cv::Point2f(ref_pts_rot_plane(0), ref_pts_rot_plane(1));

        flow_corr_avg_ += cur_flow_corr;
        all_flow_cnt++;

        grid_flow_corr_[pts_idx_to_cell_idx_[i]].flow_sum += cur_flow_corr;
        grid_flow_corr_[pts_idx_to_cell_idx_[i]].cnt++;
    }
    if (all_flow_cnt > 10) {
        flow_corr_avg_ /= all_flow_cnt;
    } else {
        flow_corr_avg_ = cv::Point2f(0, 0);
    }
    for (auto& cell : grid_flow_corr_) {
        if (cell.cnt >= 4) {
            cell.flow_avg = cell.flow_sum / cell.cnt;
        } else {
            cell.flow_avg = flow_corr_avg_;
        }
    }
}

void FeaturePredictor::inputImuMeas(const sensor_msgs::ImuConstPtr& imu_msg) {
    mtx_imu_.lock();
    imu_que_.push_back(imu_msg);
    mtx_imu_.unlock();
}

void FeaturePredictor::updateExtrinsicAndBias(const nav_msgs::OdometryPtr& msg) {
    Eigen::Quaterniond q_ic;
    Eigen::Vector3d bg;
    q_ic.x() = msg->pose.pose.orientation.x;
    q_ic.y() = msg->pose.pose.orientation.y;
    q_ic.z() = msg->pose.pose.orientation.z;
    q_ic.w() = msg->pose.pose.orientation.w;
    bg.x()   = msg->twist.twist.linear.x;
    bg.y()   = msg->twist.twist.linear.y;
    bg.z()   = msg->twist.twist.linear.z;
    mtx_ext_.lock();
    q_ic_ = q_ic;
    if (bg.norm() < 0.2) {
        bias_gyro_ = bg;
    }
    mtx_ext_.unlock();
}
