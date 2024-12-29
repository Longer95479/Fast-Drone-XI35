#include <ros/ros.h>
#include <thread>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>

#include "feature_tracker.h"
#include "line_feature_tracker.h"
#include "thread_pool/ThreadPool.h"

queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

ros::Publisher pub_img, pub_match, pub_line_img;

FeatureTracker tracker;
LineFeatureTracker line_tracker;

ThreadPool thread_pool(3);

int DETECT_LINE;

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}

void pubTrackImage(const cv::Mat &imgTrack, const double t)
{
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_match.publish(imgTrackMsg);
}

void pubLineTrackImage(const cv::Mat &imgTrack, const double t)
{
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_line_img.publish(imgTrackMsg);
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}
//freq ctrl
double first_image_time = -1;
int process_counts = 1;
bool process_this_frame = false;
void sync_process()
{
	double FREQ = tracker.feature_tracker_config.pub_freq;
	int use_opticalflow = tracker.feature_tracker_config.use_opticalflow;
    while(1)
    {
		double cur_time = 0;
		std_msgs::Header header;
		bool pub_this_frame = false;
        if(tracker.stereo_cam)
        {
            cv::Mat image0, image1;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                if(time0 < time1)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    cur_time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
					//process freq control
					if(first_image_time < 0)
					{
						first_image_time = cur_time;
						process_this_frame = false;
						img0_buf.pop();
						img1_buf.pop();
						m_buf.unlock();
						continue;
					}
					if (round(1.0 * process_counts / (cur_time - first_image_time)) <= FREQ)
					{
						process_this_frame = true;
						if (abs(1.0 * process_counts / (cur_time - first_image_time) - FREQ) < 0.01 * FREQ)
						{
							first_image_time = cur_time;
							process_counts = 0;
						}
					}
					else
						process_this_frame = false;

                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                }
            }
            m_buf.unlock();
            if(!image0.empty() && process_this_frame)
			{
				process_counts++;
				//feature track
				TicToc tic_tk;
				std::future<void> tracker_future;
				if(use_opticalflow)
					tracker_future = thread_pool.submit(std::bind(&FeatureTracker::track_img_use_opticalflow, &tracker, 
														cur_time, std::ref(image0), std::ref(image1)));
					//tracker.track_img_use_opticalflow(cur_time, image0, image1);
				else
					tracker_future = thread_pool.submit(std::bind(&FeatureTracker::track_img, &tracker, 
														cur_time, std::ref(image0), std::ref(image1)));
					//tracker.track_img(cur_time, image0, image1);
				//line track
				std:future<void> line_tracker_future;
				if(DETECT_LINE)
					line_tracker_future = thread_pool.submit(std::bind(&LineFeatureTracker::readImage, &line_tracker,
															 cur_time, std::ref(image0)));
				tracker_future.get();
				line_tracker_future.get();

				pub_this_frame = true;
				ROS_INFO("feature track stereo cost %f ms", tic_tk.toc());
			}
        }
        else
        {
            cv::Mat image;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                cur_time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
			{
                tracker.track_img(cur_time, image);
				pub_this_frame = true;
			}
        }
		//publish
		if(pub_this_frame)
		{
			sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
			sensor_msgs::ChannelFloat32 ch0;
			sensor_msgs::ChannelFloat32 ch1;
			sensor_msgs::ChannelFloat32 ch2;
			sensor_msgs::ChannelFloat32 ch3;
			sensor_msgs::ChannelFloat32 ch4;
			sensor_msgs::ChannelFloat32 ch5;
			sensor_msgs::ChannelFloat32 ch6;
			sensor_msgs::ChannelFloat32 ch7;

			feature_points->header = header;
			feature_points->header.frame_id = "world";
			//add point features
			auto &id_of_point = ch0;
			auto &camera_id_of_point = ch1;
			auto &u_of_point = ch2;
			auto &v_of_point = ch3;
			auto &velocity_x_of_point = ch4;
			auto &velocity_y_of_point = ch5;
			auto &feature_type = ch7;
			

			auto &un_pts = tracker.cur_un_pts;
            auto &cur_pts = tracker.cur_pts;
            auto &ids = tracker.cur_ids;
            auto &pts_velocity = tracker.pts_velocity;
			for(int i = 0; i < tracker.cur_ids.size(); i++)
			{
				geometry_msgs::Point32 p;
				p.x = un_pts[i].x;
				p.y = un_pts[i].y;
				p.z = 1;
				feature_points->points.push_back(p);
				id_of_point.values.push_back(ids[i]);
				camera_id_of_point.values.push_back(0);
				u_of_point.values.push_back(cur_pts[i].x);
				v_of_point.values.push_back(cur_pts[i].y);
				velocity_x_of_point.values.push_back(pts_velocity[i].x);
				velocity_y_of_point.values.push_back(pts_velocity[i].y);
				ch6.values.push_back(0);//no use
				feature_type.values.push_back(0);
			}
			if(tracker.stereo_cam)
			{
				auto &un_right_pts = tracker.cur_un_right_pts;
            	auto &right_pts = tracker.cur_right_pts;
            	auto &right_ids = tracker.right_ids;
            	auto &right_pts_velocity = tracker.right_pts_velocity;
				for(int i = 0; i < tracker.right_ids.size(); i++)
				{
					geometry_msgs::Point32 p;
					p.x = un_right_pts[i].x;
					p.y = un_right_pts[i].y;
					p.z = 1;
					feature_points->points.push_back(p);
					id_of_point.values.push_back(right_ids[i]);
					camera_id_of_point.values.push_back(1);
					u_of_point.values.push_back(right_pts[i].x);
					v_of_point.values.push_back(right_pts[i].y);
					velocity_x_of_point.values.push_back(right_pts_velocity[i].x);
					velocity_y_of_point.values.push_back(right_pts_velocity[i].y);
					ch6.values.push_back(0);//no use
					feature_type.values.push_back(0);
				}
			}
			//add line features
			if(DETECT_LINE)
			{
				auto &id_of_line = ch0;
				auto &end_x_of_line = ch1;
				auto &end_y_of_line = ch2;
				auto &start_x_vel_of_line = ch3;
				auto &start_y_vel_of_line = ch4;
				auto &end_x_vel_of_line = ch5;
				auto &end_y_vel_of_line = ch6;

				auto &line_id = line_tracker.curFrame->lineID;
				auto &line_se = line_tracker.curFrame->lineSpEpUndist;
				auto &line_vel = line_tracker.curFrame->lineVelocity;
				for(int i = 0; i < line_id.size(); i++)
				{
					geometry_msgs::Point32 start_p;
					start_p.x = line_se[i][0];
					start_p.y = line_se[i][1];
					start_p.z = 1;
					feature_points->points.push_back(start_p);

					id_of_line.values.push_back(line_id[i]);
					end_x_of_line.values.push_back(line_se[i][2]);
					end_y_of_line.values.push_back(line_se[i][3]);
					start_x_vel_of_line.values.push_back(line_vel[i][0]);
					start_y_vel_of_line.values.push_back(line_vel[i][1]);
					end_x_vel_of_line.values.push_back(line_vel[i][2]);
					end_y_vel_of_line.values.push_back(line_vel[i][3]);
					feature_type.values.push_back(1);
				}
			}

			feature_points->channels.push_back(ch0);
			feature_points->channels.push_back(ch1);
			feature_points->channels.push_back(ch2);
			feature_points->channels.push_back(ch3);
			feature_points->channels.push_back(ch4);
			feature_points->channels.push_back(ch5);
			feature_points->channels.push_back(ch6);
			feature_points->channels.push_back(ch7);

			pub_img.publish(feature_points);
			pub_this_frame = false;
			if(tracker.feature_tracker_config.show_track)
			{
				cv::Mat track_res = tracker.getTrackImage();
				if(!track_res.empty())
					pubTrackImage(track_res, header.stamp.toSec());

				cv::Mat line_track_res = line_tracker.getTrackImage();
				if(!line_track_res.empty())
					pubLineTrackImage(line_track_res, header.stamp.toSec());
			}
		}
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "feature_tracker_node");
	ros::NodeHandle nh;

	nh.param<int>("/feature_tracker_node/detect_line", DETECT_LINE, 0);

	int log_level_debug;
	nh.param<int>("/feature_tracker_node/print_debug", log_level_debug, 0);
	if(log_level_debug)
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	else
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
	//config file
	string model_path, lightglue_plugin_path;
	nh.param<string>("/feature_tracker_node/model_path", model_path, "model");
	nh.param<string>("/feature_tracker_node/lightglue_plugin_path", lightglue_plugin_path, "");
	if(argc != 2)
	{
		printf("please intput: rosrun vins vins_node [config file] \n"
				"for example: rosrun vins vins_node "
				"~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
		return 1;
	}
	string config_file = argv[1];
	printf("config_file: %s\n", argv[1]);
	printf("model_path: %s\n", model_path.c_str());
	tracker.readConfigParameter(config_file, model_path, lightglue_plugin_path);
	line_tracker.readConfigParameter(config_file);
	//prewarm for network
	tracker.prewarmForTracker();
	//thread pool
	thread_pool.init();
	//publisher
	pub_img = nh.advertise<sensor_msgs::PointCloud>("/feature_tracker/feature", 1000);
	pub_match = nh.advertise<sensor_msgs::Image>("/feature_tracker/feature_img", 1000);
	pub_line_img = nh.advertise<sensor_msgs::Image>("/feature_tracker/line_img", 1000);
	//subscriber
	ros::Subscriber sub_img0 = nh.subscribe(tracker.feature_tracker_config.image0_topic, 100, img0_callback);
	ros::Subscriber sub_img1 = nh.subscribe(tracker.feature_tracker_config.image1_topic, 100, img1_callback);
	ros::Subscriber sub_zc = nh.subscribe<sensor_msgs::PointCloud>("/vins_fusion/world_z_in_camera", 100, std::bind(&LineFeatureTracker::zAxisInCameraCallback, &line_tracker, std::placeholders::_1));

	std::thread sync_thread{sync_process};

	ros::spin();
	thread_pool.shutdown();
	return 0;
}