#include <ros/ros.h>
#include <thread>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>

#include "feature_tracker.h"

queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

ros::Publisher pub_img,pub_match;

FeatureTracker tracker;

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
				TicToc tic_tk;
				if(use_opticalflow)
					tracker.track_img_use_opticalflow(cur_time, image0, image1);
				else
					tracker.track_img(cur_time, image0, image1);
				pub_this_frame = true;
				ROS_INFO("track stereo cost %f ms", tic_tk.toc());
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
			sensor_msgs::ChannelFloat32 id_of_point;
			sensor_msgs::ChannelFloat32 camera_id_of_point;
			sensor_msgs::ChannelFloat32 u_of_point;
			sensor_msgs::ChannelFloat32 v_of_point;
			sensor_msgs::ChannelFloat32 velocity_x_of_point;
			sensor_msgs::ChannelFloat32 velocity_y_of_point;

			feature_points->header = header;
			feature_points->header.frame_id = "world";

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
				}
			}
			feature_points->channels.push_back(id_of_point);
			feature_points->channels.push_back(camera_id_of_point);
			feature_points->channels.push_back(u_of_point);
			feature_points->channels.push_back(v_of_point);
			feature_points->channels.push_back(velocity_x_of_point);
			feature_points->channels.push_back(velocity_y_of_point);

			pub_img.publish(feature_points);
			pub_this_frame = false;
			if(tracker.feature_tracker_config.show_track)
			{
				cv::Mat match_res = tracker.getTrackImage();
				if(!match_res.empty())
					pubTrackImage(match_res, header.stamp.toSec());
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
	int log_level_debug;
	nh.param<int>("/feature_tracker_node/print_debug", log_level_debug, 0);
	if(log_level_debug == 0)
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	else if(log_level_debug == 1)
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
	else if(log_level_debug == 2)
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

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
	ROS_INFO("config_file: %s\n", argv[1]);
	ROS_INFO("model_path: %s\n", model_path.c_str());
	tracker.readConfigParameter(config_file, model_path, lightglue_plugin_path);

	tracker.prewarmForTracker();
	
	pub_img = nh.advertise<sensor_msgs::PointCloud>("/feature_tracker/feature", 1000);
	pub_match = nh.advertise<sensor_msgs::Image>("/feature_tracker/feature_img",1000);

	ros::Subscriber sub_img0 = nh.subscribe(tracker.feature_tracker_config.image0_topic, 100, img0_callback);
	ros::Subscriber sub_img1 = nh.subscribe(tracker.feature_tracker_config.image1_topic, 100, img1_callback);

	std::thread sync_thread{sync_process};
	ros::spin();

	return 0;
}