#include <ros/ros.h>
#include "detect_box_pnp/bottom_camera.h"

#define EXTRACT_TRSL(odom)  odom.pose.pose.position.x,\
                            odom.pose.pose.position.y,\
                            odom.pose.pose.position.z
#define EXTRACT_ROTA(odom) odom.pose.pose.orientation.w,\
                            odom.pose.pose.orientation.x,\
                            odom.pose.pose.orientation.y,\
                            odom.pose.pose.orientation.z

void Bottom_Camera::readParamFile(const std::string &config_file)
{
  cv::FileStorage fs(config_file, cv::FileStorage::READ);
  if(!fs.isOpened())
  {
    ROS_WARN("Failed to read config file:%s", config_file);
    return;
  }
  //畸变参数
  cv::FileNode n = fs["distortion_parameters"];
  k1 = static_cast<double>(n["k1"]);
  k2 = static_cast<double>(n["k2"]);
  k3 = static_cast<double>(n["k3"]);
  p1 = static_cast<double>(n["p1"]);
  p2 = static_cast<double>(n["p2"]);
  //k
  n = fs["projection_parameters"];
  fx = static_cast<double>(n["fx"]);
  fy = static_cast<double>(n["fy"]);
  cx = static_cast<double>(n["cx"]);
  cy = static_cast<double>(n["cy"]);
  //外参
  cv::Mat cv_R, cv_T;
  fs["extrinsicRotation"] >> cv_R;
  fs["extrinsicTranslation"] >> cv_T;
  cv::cv2eigen(cv_R, R_IC);
  cv::cv2eigen(cv_T, T_IC);
  fs.release();
}

void Bottom_Camera::camera2World(const Eigen::Vector3d &in, Eigen::Vector3d &out)
{
  std::unique_lock<std::mutex> lck(pose_mtx);
  out = R_WI*(R_IC*in + T_IC) + T_WI;
}

void Bottom_Camera::updateCameraPose(const nav_msgs::Odometry &msg)
{
  std::unique_lock<std::mutex> lck(pose_mtx);
  R_WI = Eigen::Quaterniond(EXTRACT_ROTA(msg)).toRotationMatrix();
  T_WI = Eigen::Vector3d(EXTRACT_TRSL(msg));
}
