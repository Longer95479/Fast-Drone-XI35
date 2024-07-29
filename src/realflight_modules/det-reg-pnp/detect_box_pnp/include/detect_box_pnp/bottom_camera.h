#ifndef PNP_CAMERA_H
#define PNP_CAMERA_H
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <string>
#include <mutex>

class Bottom_Camera
{
public:
  //内参
  double k1, k2, k3, p1, p2;
  double fx, fy, cx, cy;
  //外参
  Eigen::Matrix3d R_IC;
  Eigen::Vector3d T_IC;
  //相机位姿
  Eigen::Matrix3d R_WI;
  Eigen::Vector3d T_WI;
  std::mutex pose_mtx;
  //读取参数
  void readParamFile(const std::string &config_file);
  //相机系转世界系
  void camera2World(const Eigen::Vector3d &in, Eigen::Vector3d &out);
  //更新相机位姿
  void updateCameraPose(const nav_msgs::Odometry &msg);

  Bottom_Camera()
  {
    R_WI << 1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0;
    T_WI << 0.0, 0.0, 0.0; 
  }
  ~Bottom_Camera(){}
};

#endif
