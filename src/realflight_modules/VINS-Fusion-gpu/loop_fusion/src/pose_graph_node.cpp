/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <mutex>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <queue>
#include <thread>
#include <vector>

#include "keyframe.h"
#include "parameters.h"
#include "pose_graph.h"
#include "superpoint/GlobalFeature.h"
#include "utility/CameraPoseVisualization.h"
#include "utility/tic_toc.h"
#define SKIP_FIRST_CNT 10
using namespace std;

struct KeyFrameMsg {
  sensor_msgs::ImageConstPtr image_ptr{nullptr};
  nav_msgs::Odometry::ConstPtr pose_ptr{nullptr};
  sensor_msgs::PointCloudConstPtr point_ptr{nullptr};
  sensor_msgs::PointCloudConstPtr line_ptr{nullptr};
  superpoint::GlobalFeatureConstPtr vlad_ptr{nullptr};
  sensor_msgs::PointCloudConstPtr xfeats_ptr{nullptr};
};

queue<sensor_msgs::ImageConstPtr> image_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<sensor_msgs::PointCloudConstPtr> point_buf;
queue<sensor_msgs::PointCloudConstPtr> line_buf;
queue<superpoint::GlobalFeatureConstPtr> vlad_buf;
queue<sensor_msgs::PointCloudConstPtr> xfeats_buf;

queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index = 0;
int sequence = 1;
PoseGraph posegraph;
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int DEBUG_IMAGE;

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
ros::Publisher pub_match_img;
ros::Publisher pub_loop_img;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_odometry_rect;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;

ros::Publisher pub_point_cloud, pub_margin_cloud;

void new_sequence() {
  printf("new sequence\n");
  sequence++;
  printf("sequence cnt %d \n", sequence);
  if (sequence > 5) {
    ROS_WARN(
        "only support 5 sequences since it's boring to copy code for more "
        "sequences.");
    ROS_BREAK();
  }
  posegraph.posegraph_visualization->reset();
  posegraph.publish();
  m_buf.lock();
  while (!image_buf.empty()) image_buf.pop();
  while (!point_buf.empty()) point_buf.pop();
  while (!pose_buf.empty()) pose_buf.pop();
  while (!odometry_buf.empty()) odometry_buf.pop();
  m_buf.unlock();
}

void image_callback(const sensor_msgs::ImageConstPtr& image_msg) {
  // ROS_INFO("image_callback!");
  m_buf.lock();
  image_buf.push(image_msg);
  m_buf.unlock();
  // printf(" image time %f \n", image_msg->header.stamp.toSec());

  // detect unstable camera stream
  if (last_image_time == -1)
    last_image_time = image_msg->header.stamp.toSec();
  else if (image_msg->header.stamp.toSec() - last_image_time > 1.0 ||
           image_msg->header.stamp.toSec() < last_image_time) {
    ROS_WARN("image discontinue! detect a new sequence!");
    new_sequence();
  }
  last_image_time = image_msg->header.stamp.toSec();
}

void point_callback(const sensor_msgs::PointCloudConstPtr& point_msg) {
  // ROS_INFO("point_callback!");
  m_buf.lock();
  point_buf.push(point_msg);
  m_buf.unlock();
  /*
  for (unsigned int i = 0; i < point_msg->points.size(); i++)
  {
      printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i ,
  point_msg->points[i].x, point_msg->points[i].y, point_msg->points[i].z,
                                                   point_msg->channels[i].values[0],
                                                   point_msg->channels[i].values[1]);
  }
  */
  // for visualization
  sensor_msgs::PointCloud point_cloud;
  point_cloud.header = point_msg->header;
  for (unsigned int i = 0; i < point_msg->points.size(); i++) {
    cv::Point3f p_3d;
    p_3d.x = point_msg->points[i].x;
    p_3d.y = point_msg->points[i].y;
    p_3d.z = point_msg->points[i].z;
    Eigen::Vector3d tmp =
        posegraph.r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) +
        posegraph.t_drift;
    geometry_msgs::Point32 p;
    p.x = tmp(0);
    p.y = tmp(1);
    p.z = tmp(2);
    point_cloud.points.push_back(p);
  }
  pub_point_cloud.publish(point_cloud);
}

void xfeats_callback(const sensor_msgs::PointCloudConstPtr& xfeat_msg) {
  m_buf.lock();
  xfeats_buf.push(xfeat_msg);
  m_buf.unlock();
}

void line_callback(const sensor_msgs::PointCloudConstPtr& line_msg) {
  m_buf.lock();
  line_buf.push(line_msg);
  m_buf.unlock();
}

void vlad_callback(const superpoint::GlobalFeatureConstPtr& vlad_msg) {
  m_buf.lock();
  vlad_buf.push(vlad_msg);
  m_buf.unlock();
}

// only for visualization
void margin_point_callback(const sensor_msgs::PointCloudConstPtr& point_msg) {
  sensor_msgs::PointCloud point_cloud;
  point_cloud.header = point_msg->header;
  for (unsigned int i = 0; i < point_msg->points.size(); i++) {
    cv::Point3f p_3d;
    p_3d.x = point_msg->points[i].x;
    p_3d.y = point_msg->points[i].y;
    p_3d.z = point_msg->points[i].z;
    Eigen::Vector3d tmp =
        posegraph.r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) +
        posegraph.t_drift;
    geometry_msgs::Point32 p;
    p.x = tmp(0);
    p.y = tmp(1);
    p.z = tmp(2);
    point_cloud.points.push_back(p);
  }
  pub_margin_cloud.publish(point_cloud);
}

void pose_callback(const nav_msgs::Odometry::ConstPtr& pose_msg) {
  // ROS_INFO("pose_callback!");
  m_buf.lock();
  pose_buf.push(pose_msg);
  m_buf.unlock();
  /*
  printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n",
  pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y,
                                                     pose_msg->pose.pose.position.z,
                                                     pose_msg->pose.pose.orientation.w,
                                                     pose_msg->pose.pose.orientation.x,
                                                     pose_msg->pose.pose.orientation.y,
                                                     pose_msg->pose.pose.orientation.z);
  */
}

void vio_callback(const nav_msgs::Odometry::ConstPtr& pose_msg) {
  // ROS_INFO("vio_callback!");
  Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y,
                 pose_msg->pose.pose.position.z);
  Quaterniond vio_q;
  vio_q.w() = pose_msg->pose.pose.orientation.w;
  vio_q.x() = pose_msg->pose.pose.orientation.x;
  vio_q.y() = pose_msg->pose.pose.orientation.y;
  vio_q.z() = pose_msg->pose.pose.orientation.z;

  vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
  vio_q = posegraph.w_r_vio * vio_q;

  vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
  vio_q = posegraph.r_drift * vio_q;

  nav_msgs::Odometry odometry;
  odometry.header = pose_msg->header;
  odometry.header.frame_id = "world";
  odometry.pose.pose.position.x = vio_t.x();
  odometry.pose.pose.position.y = vio_t.y();
  odometry.pose.pose.position.z = vio_t.z();
  odometry.pose.pose.orientation.x = vio_q.x();
  odometry.pose.pose.orientation.y = vio_q.y();
  odometry.pose.pose.orientation.z = vio_q.z();
  odometry.pose.pose.orientation.w = vio_q.w();
  pub_odometry_rect.publish(odometry);

  Vector3d vio_t_cam;
  Quaterniond vio_q_cam;
  vio_t_cam = vio_t + vio_q * tic;
  vio_q_cam = vio_q * qic;

  cameraposevisual.reset();
  cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
  cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);
}

void extrinsic_callback(const nav_msgs::Odometry::ConstPtr& pose_msg) {
  m_process.lock();
  tic = Vector3d(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y,
                 pose_msg->pose.pose.position.z);
  qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                    pose_msg->pose.pose.orientation.x,
                    pose_msg->pose.pose.orientation.y,
                    pose_msg->pose.pose.orientation.z)
            .toRotationMatrix();
  m_process.unlock();
}

std::optional<KeyFrameMsg> sync_msg() {
  std::lock_guard<std::mutex> lck(m_buf);
  if (image_buf.empty() || pose_buf.empty() || point_buf.empty() ||
      line_buf.empty() || vlad_buf.empty() || xfeats_buf.empty()) {
    return std::nullopt;
  }
  // sync pose point line first
  if (pose_buf.front()->header.stamp.toSec() !=
          point_buf.front()->header.stamp.toSec() ||
      pose_buf.front()->header.stamp.toSec() !=
          line_buf.front()->header.stamp.toSec() ||
      point_buf.front()->header.stamp.toSec() !=
          line_buf.front()->header.stamp.toSec()) {
    while (!pose_buf.empty()) pose_buf.pop();
    while (!point_buf.empty()) point_buf.pop();
    while (!line_buf.empty()) line_buf.pop();
    return std::nullopt;
  }
  KeyFrameMsg msg_collect;
  msg_collect.pose_ptr = pose_buf.front();
  pose_buf.pop();
  msg_collect.point_ptr = point_buf.front();
  point_buf.pop();
  msg_collect.line_ptr = line_buf.front();
  line_buf.pop();

  // sync image，vlad，xfeats
  double sync_stamp = msg_collect.pose_ptr->header.stamp.toSec();
  if (!(image_buf.front()->header.stamp.toSec() <= sync_stamp &&
        image_buf.back()->header.stamp.toSec() >= sync_stamp)) {
    return std::nullopt;
  }
  if (!(vlad_buf.front()->stamp.toSec() <= sync_stamp &&
        vlad_buf.back()->stamp.toSec() >= sync_stamp)) {
    return std::nullopt;
  }
  if (!(xfeats_buf.front()->header.stamp.toSec() <= sync_stamp &&
        xfeats_buf.back()->header.stamp.toSec() >= sync_stamp)) {
    return std::nullopt;
  }
  // sync image
  while (!image_buf.empty() &&
         image_buf.front()->header.stamp.toSec() <= sync_stamp) {
    msg_collect.image_ptr = image_buf.front();
    image_buf.pop();
  }
  if (msg_collect.image_ptr == nullptr ||
      msg_collect.image_ptr->header.stamp.toSec() != sync_stamp) {
    return std::nullopt;
  }
  // sync vlad
  while (!vlad_buf.empty() && vlad_buf.front()->stamp.toSec() <= sync_stamp) {
    msg_collect.vlad_ptr = vlad_buf.front();
    vlad_buf.pop();
  }
  if (msg_collect.vlad_ptr == nullptr ||
      msg_collect.vlad_ptr->stamp.toSec() != sync_stamp) {
    return std::nullopt;
  }
  // sync xfeats
  while (!xfeats_buf.empty() &&
         xfeats_buf.front()->header.stamp.toSec() <= sync_stamp) {
    msg_collect.xfeats_ptr = xfeats_buf.front();
    xfeats_buf.pop();
  }
  if (msg_collect.xfeats_ptr == nullptr ||
      msg_collect.xfeats_ptr->header.stamp.toSec() != sync_stamp) {
    return std::nullopt;
  }

  return msg_collect;
}

void process() {
  while (true) {
    // find out the messages with same time stamp
    auto msg_collect = sync_msg();

    if (msg_collect.has_value()) {
      // skip fisrt few
      if (skip_first_cnt < SKIP_FIRST_CNT) {
        skip_first_cnt++;
        continue;
      }

      if (skip_cnt < SKIP_CNT) {
        skip_cnt++;
        continue;
      } else {
        skip_cnt = 0;
      }
      auto& [image_msg, pose_msg, point_msg, line_msg, vlad_msg, xfeats_msg] =
          msg_collect.value();
      cv_bridge::CvImageConstPtr ptr;
      if (image_msg->encoding == "8UC1") {
        sensor_msgs::Image img;
        img.header = image_msg->header;
        img.height = image_msg->height;
        img.width = image_msg->width;
        img.is_bigendian = image_msg->is_bigendian;
        img.step = image_msg->step;
        img.data = image_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
      } else
        ptr =
            cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

      cv::Mat image = ptr->image;
      // build keyframe
      Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                            pose_msg->pose.pose.position.y,
                            pose_msg->pose.pose.position.z);
      Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                               pose_msg->pose.pose.orientation.x,
                               pose_msg->pose.pose.orientation.y,
                               pose_msg->pose.pose.orientation.z)
                       .toRotationMatrix();
      if ((T - last_t).norm() > SKIP_DIS) {
        vector<cv::Point3f> point_3d;
        vector<cv::Point2f> point_2d_uv;
        vector<cv::Point2f> point_2d_normal;
        vector<double> point_id;
        vector<XDescType> point_xdesc;
        for (unsigned int i = 0; i < point_msg->points.size(); i++) {
          cv::Point3f p_3d;
          p_3d.x = point_msg->points[i].x;
          p_3d.y = point_msg->points[i].y;
          p_3d.z = point_msg->points[i].z;
          point_3d.push_back(p_3d);

          cv::Point2f p_2d_uv, p_2d_normal;
          double p_id;
          XDescType xdesc;
          p_id = point_msg->channels[0].values[i];
          p_2d_normal.x = point_msg->channels[1].values[i];
          p_2d_normal.y = point_msg->channels[2].values[i];
          p_2d_uv.x = point_msg->channels[3].values[i];
          p_2d_uv.y = point_msg->channels[4].values[i];
          for (int j = 5; j < 69; j++) {
            xdesc(j - 5, 0) = point_msg->channels[j].values[i];
          }

          point_2d_normal.push_back(p_2d_normal);
          point_2d_uv.push_back(p_2d_uv);
          point_id.push_back(p_id);
          point_xdesc.push_back(xdesc);
          // printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
        }

        vector<cv::Point2f> xfeats_uv;
        vector<cv::Point2f> xfeats_norm;
        vector<XDescType> xfeats_desc;
        for (unsigned int i = 0; i < xfeats_msg->points.size(); i++) {
          cv::Point2f pt_uv, pt_norm;
          pt_norm.x = xfeats_msg->points[i].x;
          pt_norm.y = xfeats_msg->points[i].y;
          pt_uv.x = xfeats_msg->channels[0].values[i];
          pt_uv.y = xfeats_msg->channels[1].values[i];
          XDescType pt_desc;
          for (int j = 2; j < 66; j++) {
            pt_desc(j - 2, 0) = xfeats_msg->channels[j].values[i];
          }
          xfeats_uv.push_back(pt_uv);
          xfeats_norm.push_back(pt_norm);
          xfeats_desc.push_back(pt_desc);
        }

        VladType vlad_desc;
        for (int j = 0; j < 4096; j++) {
          vlad_desc(j, 0) = vlad_msg->descriptor[j];
        }
        // ROS_INFO("Add keyframe %d, %f, has %d points.", frame_index,
        //          pose_msg->header.stamp.toSec(), xfeats_msg->points.size());
        KeyFramePtr keyframe_ptr =
            KeyFrame::create(pose_msg->header.stamp.toSec(), frame_index, T, R,
                             image, point_3d, point_2d_uv, point_2d_normal,
                             point_id, point_xdesc, vlad_desc, sequence);
        keyframe_ptr->setXFeatsPoints(xfeats_uv, xfeats_norm, xfeats_desc);
        m_process.lock();
        start_flag = 1;
        posegraph.addKeyFrame(keyframe_ptr, 1);
        m_process.unlock();
        frame_index++;
        last_t = T;
      }
    }
    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
  }
}

void command() {
  while (1) {
    char c = getchar();
    if (c == 's') {
      m_process.lock();
      // posegraph.savePoseGraph();
      posegraph.onlySaveImage();
      m_process.unlock();
      printf(
          "save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 "
          "in the config file to reuse it next time\n");
      printf("program shutting down...\n");
      ros::shutdown();
    }
    if (c == 'n') new_sequence();

    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "loop_fusion");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);
  posegraph.registerPub(n);

  VISUALIZATION_SHIFT_X = 0;
  VISUALIZATION_SHIFT_Y = 0;
  SKIP_CNT = 0;
  SKIP_DIS = 0;

  if (argc != 2) {
    printf(
        "please intput: rosrun loop_fusion loop_fusion_node [config file] \n"
        "for example: rosrun loop_fusion loop_fusion_node "
        "/home/tony-ws1/catkin_ws/src/VINS-Fusion/config/euroc/"
        "euroc_stereo_imu_config.yaml \n");
    return 0;
  }

  string config_file = argv[1];
  printf("config_file: %s\n", argv[1]);

  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  cameraposevisual.setScale(0.1);
  cameraposevisual.setLineWidth(0.01);

  std::string IMAGE_TOPIC;
  int LOAD_PREVIOUS_POSE_GRAPH;

  ROW = fsSettings["image_height"];
  COL = fsSettings["image_width"];
  std::string pkg_path = ros::package::getPath("loop_fusion");
  string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
  cout << "vocabulary_file" << vocabulary_file << endl;
  posegraph.loadVocabulary(vocabulary_file);

  BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
  cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;

  int pn = config_file.find_last_of('/');
  std::string configPath = config_file.substr(0, pn);
  std::string cam0Calib;
  fsSettings["cam0_calib"] >> cam0Calib;
  std::string cam0Path = configPath + "/" + cam0Calib;
  printf("cam calib path: %s\n", cam0Path.c_str());
  m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
      cam0Path.c_str());

  fsSettings["image0_topic"] >> IMAGE_TOPIC;
  fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
  fsSettings["output_path"] >> VINS_RESULT_PATH;
  fsSettings["save_image"] >> DEBUG_IMAGE;

  LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
  VINS_RESULT_PATH = VINS_RESULT_PATH + "/vio_loop.csv";
  std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
  fout.close();

  int USE_IMU = fsSettings["imu"];
  posegraph.setIMUFlag(USE_IMU);
  fsSettings.release();

  if (LOAD_PREVIOUS_POSE_GRAPH) {
    printf("load pose graph\n");
    m_process.lock();
    posegraph.loadPoseGraph();
    m_process.unlock();
    printf("load pose graph finish\n");
    load_flag = 1;
  } else {
    printf("no previous pose graph\n");
    load_flag = 1;
  }

  ros::Subscriber sub_vio =
      n.subscribe("/vins_fusion/odometry", 2000, vio_callback);
  ros::Subscriber sub_image = n.subscribe(IMAGE_TOPIC, 2000, image_callback);
  ros::Subscriber sub_pose =
      n.subscribe("/vins_fusion/keyframe_pose", 2000, pose_callback);
  ros::Subscriber sub_extrinsic =
      n.subscribe("/vins_fusion/extrinsic", 2000, extrinsic_callback);
  ros::Subscriber sub_point =
      n.subscribe("/vins_fusion/keyframe_point", 2000, point_callback);
  ros::Subscriber sub_line =
      n.subscribe("/vins_fusion/keyframe_line", 2000, line_callback);
  ros::Subscriber sub_vlad =
      n.subscribe("/feature_tracker/global_feature", 2000, vlad_callback);
  ros::Subscriber sub_xfeats =
      n.subscribe("/feature_tracker/xfeats", 2000, xfeats_callback);
  ros::Subscriber sub_margin_point =
      n.subscribe("/vins_fusion/margin_cloud", 2000, margin_point_callback);

  pub_match_img = n.advertise<sensor_msgs::Image>("match_image", 1000);
  pub_loop_img = n.advertise<sensor_msgs::Image>("loop_image", 1000);
  pub_camera_pose_visual =
      n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
  pub_point_cloud =
      n.advertise<sensor_msgs::PointCloud>("point_cloud_loop_rect", 1000);
  pub_margin_cloud =
      n.advertise<sensor_msgs::PointCloud>("margin_cloud_loop_rect", 1000);
  pub_odometry_rect = n.advertise<nav_msgs::Odometry>("odometry_rect", 1000);

  std::thread measurement_process;
  std::thread keyboard_command_process;

  measurement_process = std::thread(process);
  keyboard_command_process = std::thread(command);

  ros::spin();

  return 0;
}
