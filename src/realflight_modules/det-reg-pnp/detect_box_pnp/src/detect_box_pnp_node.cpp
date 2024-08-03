#include <time.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <numeric>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <detect_box_pnp/DetectInfo.h>
#include <detect_box_pnp/ProcessImage.h>

#include "detect_box_pnp/printf_utils.h"
#include "detect_box_pnp/bottom_camera.h"


using namespace std;
using namespace cv;
detect_box_pnp::ProcessImage target_recognition_srv;
// 输入图像
image_transport::Subscriber image_sub;
// 发布目标在相机系下位置
ros::Publisher target_position_pub;
ros::Publisher targetInWorld_pub;
// 发布识别后的图像
image_transport::Publisher detect_result_image_pub;

ros::Subscriber drone_position_world_frame_sub;

Mat img;

geometry_msgs::PoseStamped target_position_world_frame_msg;

sensor_msgs::ImagePtr detect_result_image;

float camera_offset[3];

float detect_weight_thre;

double calculation_time;

// 相机话题中的图像同步相关变量
int frame_width, frame_height;
cv::Mat cam_image_copy;
boost::shared_mutex mutex_image_callback;
bool image_status = false;
boost::shared_mutex mutex_image_status;

Bottom_Camera bt_Camera;


// 图像接收回调函数，接收cam的话题，并将图像保存在cam_image_copy中
void cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cam_image;

    try
    {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        PCOUT(0, RED, std::string("cv_bridge exception:") + e.what());
        return;
    }

    if (cam_image)
    {
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutex_image_callback);
            cam_image_copy = cam_image->image.clone();
        }
        {
            boost::unique_lock<boost::shared_mutex> lockImageStatus(mutex_image_status);
            image_status = true;
        }
        frame_width = cam_image->image.size().width;
        frame_height = cam_image->image.size().height;
    }
    return;
}

void drone_position_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    bt_Camera.updateCameraPose(*msg);
}

// 用此函数查看是否收到图像话题
bool getImageStatus(void)
{
    boost::shared_lock<boost::shared_mutex> lock(mutex_image_status);
    return image_status;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_box_pnp");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    std::string config_file;
    nh.getParam("config_file", config_file);
    bt_Camera.readParamFile(config_file);

    std::string camera_topic;
    int detect_hz;
    nh.getParam("camera_topic", camera_topic);
    nh.getParam("detect_hz", detect_hz);
    YAML::Node camera_config = YAML::LoadFile(config_file);
    //方形目标边长
    double target_len = camera_config["target_len"].as<double>();

    nh.param<float>("detect_weight_thre", detect_weight_thre, 0.0);

    // 目标位置
    //target_position_pub = nh.advertise<detect_box_pnp::DetectInfo>("/target_position", 10);
    targetInWorld_pub = nh.advertise<geometry_msgs::PoseStamped>("/detect_box_pnp/target", 10);
    // 相机图像
    image_sub = it.subscribe(camera_topic.c_str(), 1, cameraCallback);
    // 检测结果图像
    detect_result_image_pub = it.advertise("/detect_result_image", 1);
    drone_position_world_frame_sub = nh.subscribe<nav_msgs::Odometry>("/vins_fusion/imu_propagate", 10, drone_position_cb);
    // 目标识别
    ros::ServiceClient client = nh.serviceClient<detect_box_pnp::ProcessImage>("/process_image");
    


    Mat camera_matrix;
    camera_matrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
    camera_matrix.ptr<double>(0)[0] = bt_Camera.fx;
    camera_matrix.ptr<double>(0)[2] = bt_Camera.cx;
    camera_matrix.ptr<double>(1)[1] = bt_Camera.fy;
    camera_matrix.ptr<double>(1)[2] = bt_Camera.cy;
    camera_matrix.ptr<double>(2)[2] = 1.0f;

    Mat distortion_coefficients;
    distortion_coefficients = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
    distortion_coefficients.ptr<double>(0)[0] = bt_Camera.k1;
    distortion_coefficients.ptr<double>(1)[0] = bt_Camera.k2;
    distortion_coefficients.ptr<double>(2)[0] = bt_Camera.p1;
    distortion_coefficients.ptr<double>(3)[0] = bt_Camera.p2;
    distortion_coefficients.ptr<double>(4)[0] = bt_Camera.k3;

    // ArUco Marker字典选择
    Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // 节点运行频率：1Hz
    ros::Rate loopRate(detect_hz);

    while (ros::ok())
    {
        ros::spinOnce();
        if (!getImageStatus() && ros::ok())
        {
            PCOUT(-1, WHITE, "Waiting for image");
            continue;
        }
        PCOUT(-1, GREEN, "RUNING ...");

        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutex_image_callback);
            img = cam_image_copy.clone();
        }

        auto start = chrono::high_resolution_clock::now();

        // markerids存储每个识别到二维码的编号，markerCorners每个二维码对应的四个角点的像素坐标
        std::vector<int> markerids1, markerids_deted, markerids2, markerids3;
        vector<vector<Point2f>> markerCorners1, markerCorners_deted, rejectedCandidate, markerCorners2, markerCorners3;

        // Aruco识别参数
        Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        
        //minMarkerPerimeterRate: determine minimum perimeter for marker contour to be detected. This is defined as a rate respect to the maximum dimension of the input image (default 0.03).
        parameters->minMarkerPerimeterRate=0.03;
        //maxMarkerPerimeterRate: determine maximum perimeter for marker contour to be detected. This is defined as a rate respect to the maximum dimension of the input image (default 4.0).
        parameters->maxMarkerPerimeterRate=4;
        parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
        
        cv::aruco::detectMarkers(img, dictionary, markerCorners_deted, markerids_deted, parameters, rejectedCandidate);
        cv_bridge::CvImage cv_image;
        Mat resultImg;
        vector<Vec3d> rvecs, tvecs;
        for (size_t i = 0; i < markerCorners_deted.size(); ++i) 
        {
            // 获取当前检测到的角点
            vector<Point2f> corners = markerCorners_deted[i];

            // 定义正视图的目标角点
            vector<Point2f> squareCorners = {
                    Point2f(0, 0),
                    Point2f(100, 0),
                    Point2f(100, 100),
                    Point2f(0, 100)
            };

            // 计算透视变换矩阵
            Mat transformation = getPerspectiveTransform(corners, squareCorners);

            // 应用透视变换以获取正视图
            warpPerspective(img, resultImg, transformation, Size(100, 100), INTER_NEAREST);
        
            cv_image.image = resultImg;
            cv_image.encoding = "bgr8";
            cv_image.toImageMsg(target_recognition_srv.request.image);
            client.call(target_recognition_srv);

            if (target_recognition_srv.response.prediction == 1 && target_recognition_srv.response.weight > weight1 && target_recognition_srv.response.weight > detect_weight_thre)
            {
                weight1 = target_recognition_srv.response.weight;
                markerCorners1.clear();
                markerids1.clear();
                markerCorners1.push_back(markerCorners_deted[i]);
                markerids1.push_back(1);
            }
            if (target_recognition_srv.response.prediction == 2 && target_recognition_srv.response.weight > weight2 && target_recognition_srv.response.weight > detect_weight_thre)
            {
                weight2 = target_recognition_srv.response.weight;
                markerCorners2.clear();
                markerids2.clear();
                markerCorners2.push_back(markerCorners_deted[i]);
                markerids2.push_back(2);
            }
            if (target_recognition_srv.response.prediction == 3 && target_recognition_srv.response.weight > weight3 && target_recognition_srv.response.weight > detect_weight_thre)
            {
                weight3 = target_recognition_srv.response.weight;
                markerCorners3.clear();
                markerids3.clear();
                markerCorners3.push_back(markerCorners_deted[i]);
                markerids3.push_back(3);
            }
        }
        if (!markerids1.empty())
        {
            // 可视化，绘制方框
            aruco::drawDetectedMarkers(img, markerCorners1, markerids1);

            double id_to8_t[3];
            id_to8_t[0] = 0.;
            id_to8_t[1] = 0.;
            id_to8_t[2] = 0.;
            
            //PnP
            aruco::estimatePoseSingleMarkers(markerCorners1, target_len, camera_matrix, distortion_coefficients, rvecs, tvecs);
            
            // 可视化，绘制坐标轴
            aruco::drawAxis(img, camera_matrix, distortion_coefficients, rvecs[0], tvecs[0], target_len * 0.5f);

            // 姿态
            cv::Mat rotation_matrix;
            cv::Rodrigues(rvecs[0], rotation_matrix);
            Eigen::Matrix3d rotation_matrix_eigen;
            cv::cv2eigen(rotation_matrix, rotation_matrix_eigen);
            Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix_eigen);
            q.normalize();

            // 位置
            std::vector<double> vec_t{tvecs[0][0], tvecs[0][1], tvecs[0][2]};
            cv::Mat vec_t_mat{vec_t};
            vec_t_mat = vec_t_mat;
            vec_t_mat.convertTo(vec_t_mat, CV_32FC1);

            // 原点位置
            cv::Mat id_to8_t_mat = cv::Mat(3, 1, CV_32FC1, id_to8_t);
            rotation_matrix.convertTo(rotation_matrix, CV_32FC1);
            cv::Mat id_8_t = rotation_matrix * id_to8_t_mat + vec_t_mat;
            
            // 整合数据发布
            // 相机坐标系下目标中心点位置
            float o_tx = id_8_t.at<float>(0);
            float o_ty = id_8_t.at<float>(1);
            float o_tz = id_8_t.at<float>(2);
            Eigen::Vector3d target_inCamera(o_tx, o_ty, o_tz);
            Eigen::Vector3d target_inWorld;
            bt_Camera.camera2World(target_inCamera, target_inWorld);
            
            target_position_world_frame_msg.header.seq = 1;
            target_position_world_frame_msg.header.stamp = ros::Time::now();
            target_position_world_frame_msg.pose.position.x = target_inWorld.x();
            target_position_world_frame_msg.pose.position.y = target_inWorld.y();
            target_position_world_frame_msg.pose.position.z = 1;
            targetInWorld_pub.publish(target_position_world_frame_msg);
        }
        if (!markerids2.empty())
        {
            // 可视化，绘制方框
            aruco::drawDetectedMarkers(img, markerCorners2, markerids2);

            double id_to8_t[3];
            id_to8_t[0] = 0.;
            id_to8_t[1] = 0.;
            id_to8_t[2] = 0.;
            
            //PnP
            aruco::estimatePoseSingleMarkers(markerCorners2, target_len, camera_matrix, distortion_coefficients, rvecs, tvecs);
            
            // 可视化，绘制坐标轴
            aruco::drawAxis(img, camera_matrix, distortion_coefficients, rvecs[0], tvecs[0], target_len * 0.5f);

            // 姿态
            cv::Mat rotation_matrix;
            cv::Rodrigues(rvecs[0], rotation_matrix);
            Eigen::Matrix3d rotation_matrix_eigen;
            cv::cv2eigen(rotation_matrix, rotation_matrix_eigen);
            Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix_eigen);
            q.normalize();

            // 位置
            std::vector<double> vec_t{tvecs[0][0], tvecs[0][1], tvecs[0][2]};
            cv::Mat vec_t_mat{vec_t};
            vec_t_mat = vec_t_mat;
            vec_t_mat.convertTo(vec_t_mat, CV_32FC1);

            // 原点位置
            cv::Mat id_to8_t_mat = cv::Mat(3, 1, CV_32FC1, id_to8_t);
            rotation_matrix.convertTo(rotation_matrix, CV_32FC1);
            cv::Mat id_8_t = rotation_matrix * id_to8_t_mat + vec_t_mat;
            
            // 整合数据发布
            // 相机坐标系下目标中心点位置
            float o_tx = id_8_t.at<float>(0);
            float o_ty = id_8_t.at<float>(1);
            float o_tz = id_8_t.at<float>(2);
            Eigen::Vector3d target_inCamera(o_tx, o_ty, o_tz);
            Eigen::Vector3d target_inWorld;
            bt_Camera.camera2World(target_inCamera, target_inWorld);

            target_position_world_frame_msg.header.seq = 2;
            target_position_world_frame_msg.header.stamp = ros::Time::now();
            target_position_world_frame_msg.pose.position.x = target_inWorld.x();
            target_position_world_frame_msg.pose.position.y = target_inWorld.y();
            target_position_world_frame_msg.pose.position.z = 2;
            targetInWorld_pub.publish(target_position_world_frame_msg);
        }
        if (!markerids3.empty())
        {
            // 可视化，绘制方框
            aruco::drawDetectedMarkers(img, markerCorners3, markerids3);

            double id_to8_t[3];
            id_to8_t[0] = 0.;
            id_to8_t[1] = 0.;
            id_to8_t[2] = 0.;
            
            //PnP
            aruco::estimatePoseSingleMarkers(markerCorners3, target_len, camera_matrix, distortion_coefficients, rvecs, tvecs);
            
            // 可视化，绘制坐标轴
            aruco::drawAxis(img, camera_matrix, distortion_coefficients, rvecs[0], tvecs[0], target_len * 0.5f);

            // 姿态
            cv::Mat rotation_matrix;
            cv::Rodrigues(rvecs[0], rotation_matrix);
            Eigen::Matrix3d rotation_matrix_eigen;
            cv::cv2eigen(rotation_matrix, rotation_matrix_eigen);
            Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix_eigen);
            q.normalize();

            // 位置
            std::vector<double> vec_t{tvecs[0][0], tvecs[0][1], tvecs[0][2]};
            cv::Mat vec_t_mat{vec_t};
            vec_t_mat = vec_t_mat;
            vec_t_mat.convertTo(vec_t_mat, CV_32FC1);

            // 原点位置
            cv::Mat id_to8_t_mat = cv::Mat(3, 1, CV_32FC1, id_to8_t);
            rotation_matrix.convertTo(rotation_matrix, CV_32FC1);
            cv::Mat id_8_t = rotation_matrix * id_to8_t_mat + vec_t_mat;
            
            // 整合数据发布
            // 相机坐标系下目标中心点位置
            float o_tx = id_8_t.at<float>(0);
            float o_ty = id_8_t.at<float>(1);
            float o_tz = id_8_t.at<float>(2);
            Eigen::Vector3d target_inCamera(o_tx, o_ty, o_tz);
            Eigen::Vector3d target_inWorld;
            bt_Camera.camera2World(target_inCamera, target_inWorld);

            target_position_world_frame_msg.header.seq = 3;
            target_position_world_frame_msg.header.stamp = ros::Time::now();
            target_position_world_frame_msg.pose.position.x = target_inWorld.x();
            target_position_world_frame_msg.pose.position.y = target_inWorld.y();
            target_position_world_frame_msg.pose.position.z = 3;
            targetInWorld_pub.publish(target_position_world_frame_msg);
        }

        detect_result_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        detect_result_image_pub.publish(detect_result_image);
        auto finish = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(finish - start);
        cout << duration.count() << "ms" << endl;

        loopRate.sleep();
    }
}
