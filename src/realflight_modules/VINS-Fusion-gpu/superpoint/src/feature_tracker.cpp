#include "feature_tracker.h"

double distance(cv::Point2f pt1, cv::Point2f pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

template<typename T>
void reduceVector(vector<T> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
        	v[j++] = v[i];
    v.resize(j);
}

bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    int BORDER_SIZE = feature_tracker_config.borders;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < feature_tracker_config.col - BORDER_SIZE && \
		BORDER_SIZE <= img_y && img_y < feature_tracker_config.row - BORDER_SIZE;
}

vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;
}

vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &_cur_ids, vector<cv::Point2f> &_cur_un_pts, 
                                            unordered_map<int, cv::Point2f> &cur_id_pts, unordered_map<int, cv::Point2f> &prev_id_pts)

{
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < _cur_ids.size(); i++)
    {
		cur_id_pts[_cur_ids[i]] = _cur_un_pts[i];
    }

    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        double dt = cur_time - prev_time;
        
        for (unsigned int i = 0; i < _cur_ids.size(); i++)
        {
            std::unordered_map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(_cur_ids[i]);
            if (it != prev_id_pts.end())
            {
                double v_x = (_cur_un_pts[i].x - it->second.x) / dt;
                double v_y = (_cur_un_pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else
                pts_velocity.push_back(cv::Point2f(0, 0));

        }
    }
    else
    {
        for (unsigned int i = 0; i < _cur_un_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity;
}

void FeatureTracker::track_img(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
	cur_time = _cur_time;
	cur_img = _img;
	//clear current pts
	cur_pts.clear();
	cur_ids.clear();
	cur_un_pts.clear();
	cur_features.setZero();

	cout << "*********** current frame ***********" << endl;
	TicToc tic_d;
	///detect points and extract desc
	if(!feature_detector->Detect(cur_img, cur_features))
	{
		ROS_ERROR("error occured when extract features!");
		prev_img = cur_img;
		prev_pts = cur_pts;
		prev_ids = cur_ids;
		prev_un_pts = cur_un_pts;
		prev_time = cur_time;
		return;
	}
	int cur_detected_size = cur_features.cols();
	ROS_DEBUG("detect %d features in left image, cost %f ms.", cur_detected_size, tic_d.toc());

	for(int i = 0; i < cur_detected_size; i++)
	{
		cur_pts.emplace_back(cur_features(1, i), cur_features(2, i));
		if(first_image_flag)
		{
			cur_ids.push_back(n_id);
			n_id++;
		}
		else
			cur_ids.push_back(-1);
	}
	if(first_image_flag)
		first_image_flag = false;

	if(prev_pts.size() > 0)
	{
		TicToc tic_m;
		//match to prev
		vector<cv::DMatch> matches;
		point_matcher->MatchingPoints(cur_features, prev_features, matches, true);
		ROS_DEBUG("match size %d, cost %f ms.", matches.size(), tic_m.toc());
		//process ids
		for(auto &match : matches)
		{
			ROS_ASSERT(match.queryIdx < cur_ids.size() && match.trainIdx < prev_ids.size());
			cur_ids[match.queryIdx] = prev_ids[match.trainIdx];
		}
		//separate unmatched points
		TicToc tic_s;
		int matched_counts = 0, unmatched_counts = 0;
		vector<cv::Point2f> unmatched_pts;
		Eigen::Matrix<float, 259, Eigen::Dynamic> unmatched_features;
		unmatched_features.resize(259, cur_detected_size);
		for(int i = 0; i < cur_detected_size; i++)
		{	
			if(cur_ids[i] == -1)
			{//unmatched points
				unmatched_pts.emplace_back(cur_pts[i].x, cur_pts[i].y);
				unmatched_features.col(unmatched_counts) = cur_features.col(i);
				unmatched_counts++;
			}
			else
			{
				cur_pts[matched_counts] = cur_pts[i];
				cur_ids[matched_counts] = cur_ids[i];
				cur_features.col(matched_counts) = cur_features.col(i);
				matched_counts++;
			}
		}
		cur_pts.resize(matched_counts);
		cur_ids.resize(matched_counts);
		ROS_DEBUG("separate unmatched points cost %f ms, matched_counts is %d, unmatched_counts is %d.", 
				tic_s.toc(), matched_counts, unmatched_counts);
		//add new points if matched points size below threshold
		int diff_n = feature_tracker_config.max_cnt - matched_counts;
		int final_counts = matched_counts;
		if(diff_n > 0)
		{//need to add new pts
			for(int i = 0; i < diff_n && final_counts < cur_detected_size; i++)
			{
				cur_pts.emplace_back(unmatched_pts[i].x, unmatched_pts[i].y);
				cur_ids.push_back(n_id++);
				cur_features.col(final_counts++) = unmatched_features.col(i);
			}
		}
		ROS_DEBUG("final cur pts size is %d.", final_counts);
		cur_features.resize(259, final_counts);		
	}
	//undistorted
	cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
	//calculate the velocity of cur_un_pts
	pts_velocity = ptsVelocity(cur_ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

	/***process the right image***/
	if(!_img1.empty() && stereo_cam)
	{
		right_img = _img1;
		right_ids.clear();
		cur_right_pts.clear();
		cur_un_right_pts.clear();
		right_pts_velocity.clear();
		cur_un_right_pts_map.clear();
		if(!cur_pts.empty())
		{
			if(feature_tracker_config.use_opticalflow_stereo)
			{//use opeicalflow
				vector<cv::Point2f> reverseLeftPts;
            	vector<uchar> status, statusRightLeft;
				TicToc t_og1;
				cv::cuda::GpuMat cur_gpu_img(cur_img);
				cv::cuda::GpuMat right_gpu_Img(right_img);
				cv::cuda::GpuMat cur_gpu_pts(cur_pts);
				cv::cuda::GpuMat cur_right_gpu_pts;
				cv::cuda::GpuMat gpu_status;
				cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(
				cv::Size(21, 21), 3, 30, false);
				d_pyrLK_sparse->calc(cur_gpu_img, right_gpu_Img, cur_gpu_pts, cur_right_gpu_pts, gpu_status);

				vector<cv::Point2f> tmp_cur_right_pts(cur_right_gpu_pts.cols);
				cur_right_gpu_pts.download(tmp_cur_right_pts);
				cur_right_pts = tmp_cur_right_pts;

				vector<uchar> tmp_status(gpu_status.cols);
				gpu_status.download(tmp_status);
				status = tmp_status;
				//flow back
				cv::cuda::GpuMat reverseLeft_gpu_Pts;
				cv::cuda::GpuMat status_gpu_RightLeft;
				d_pyrLK_sparse->calc(right_gpu_Img, cur_gpu_img, cur_right_gpu_pts, reverseLeft_gpu_Pts, status_gpu_RightLeft);

				vector<cv::Point2f> tmp_reverseLeft_Pts(reverseLeft_gpu_Pts.cols);
				reverseLeft_gpu_Pts.download(tmp_reverseLeft_Pts);
				reverseLeftPts = tmp_reverseLeft_Pts;

				vector<uchar> tmp1_status(status_gpu_RightLeft.cols);
				status_gpu_RightLeft.download(tmp1_status);
				statusRightLeft = tmp1_status;
				for(size_t i = 0, j = 0; i < status.size(); i++)
				{
					if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
					{
						status[i] = 1;
					}
					else
						status[i] = 0;
				}
				right_ids = cur_ids;
				reduceVector(cur_right_pts, status);
				reduceVector(right_ids, status);
				ROS_DEBUG("opticalflow for right image  tracked %d features, cost %f ms", right_ids.size(), t_og1.toc());
			}
			else
			{//use superpoint & lightglue
				TicToc tic_dr;
				cur_right_features.setZero();
				if(feature_detector->Detect(right_img, cur_right_features))
				{
					int right_pts_size = cur_right_features.cols();
					ROS_DEBUG("detect %d features in right image, cost %f ms.", right_pts_size, tic_dr.toc());
					for(int i = 0; i < right_pts_size; i++)
					{
						cur_right_pts.emplace_back(cur_right_features(1, i), cur_right_features(2, i));
						right_ids.push_back(-1);
					}
					//match to left
					TicToc tic_mr;
					vector<cv::DMatch> matches;
					point_matcher->MatchingPoints(cur_right_features, cur_features, matches, true);
					ROS_DEBUG("right match size %d, cost %f ms.", matches.size(), tic_mr.toc());
					//process id
					for(auto &match : matches)
					{
						ROS_ASSERT(match.queryIdx < right_ids.size() && match.trainIdx < cur_ids.size());
						right_ids[match.queryIdx] = cur_ids[match.trainIdx];
					}
					//reduce the unmatched points
					int right_matched_counts = 0;
					for(int i = 0; i < right_pts_size; i++)
					{
						if(right_ids[i] != -1)
						{
							cur_right_pts[right_matched_counts] = cur_right_pts[i];
							right_ids[right_matched_counts] = right_ids[i];
							right_matched_counts++;
						}
					}
					cur_right_pts.resize(right_matched_counts);
					right_ids.resize(right_matched_counts);
				}
				else
					ROS_ERROR("error occured when extract right image features!");	
			}
			//undistorted right points
			cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
			//calculate the right points' velocity
			right_pts_velocity = ptsVelocity(right_ids, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
		}
		prev_un_right_pts_map = cur_un_right_pts_map;
	}
	calTrackCnt();
	//draw
	switch (feature_tracker_config.show_track)
	{
	case 1:
		if(!prev_img.empty() && !cur_img.empty())
			DrawMatches(prev_img, cur_img, prev_pts, cur_pts, prev_ids, cur_ids);
		break;
	case 2:
		if(!cur_img.empty() && !right_img.empty())
			DrawMatches(cur_img, right_img, cur_pts, cur_right_pts, cur_ids, right_ids);
	case 3:
		if(!prev_img.empty() && !cur_img.empty())
			DrawTrackCnt(cur_img, cur_pts, cur_ids, cur_trackcnt_umap);
	default:
		break;
	}

	int good_track_cnt = 0;
	for(auto ele : cur_trackcnt_umap)
	{
		if(ele.second >= 4)
			good_track_cnt++;
	}
	printf("good track cnt is %d.\n", good_track_cnt);

	prev_img = cur_img;
	prev_pts = cur_pts;
	prev_ids = cur_ids;
	prev_features = cur_features;
	prev_un_pts = cur_un_pts;
	prev_un_pts_map = cur_un_pts_map;
	prev_time = cur_time;
}

void FeatureTracker::DrawTrackCnt(const cv::Mat& image, const vector<cv::Point2f>& pts, const vector<int>& ids, const unordered_map<int, int>& id_cnt_umap)
{
	cv::Mat rgba_image;
	cv::cvtColor(image, rgba_image, cv::COLOR_BGR2BGRA);
	for(int i = 0; i < pts.size(); i++)
	{
		int track_cnt;
		auto it = id_cnt_umap.find(ids[i]);
		if(it == id_cnt_umap.end())
			track_cnt = 0;
		else
			track_cnt = it->second;
		double len = std::min(1.0, 1.0 * track_cnt / 20);
		cv::circle(rgba_image, pts[i], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
	}
	cv::cvtColor(rgba_image, imTrack, cv::COLOR_BGRA2BGR);
}

void FeatureTracker::DrawMatches(const cv::Mat& ref_image, const cv::Mat& image, 
									const vector<cv::Point2f>& ref_pts, const vector<cv::Point2f>& pts,
									const vector<int>& ref_ids, const vector<int>& ids)
{
	unordered_map<int, cv::Point2f> ref_ids_pts;
	for(int i = 0; i < ref_pts.size(); i++)
	{
		ref_ids_pts[ref_ids[i]] = ref_pts[i];
	}
	cv::Mat merged_image;
  	cv::hconcat(ref_image, image, merged_image);
  	cv::Mat rgba_image;
  	cv::cvtColor(merged_image, rgba_image, cv::COLOR_BGR2BGRA);
	for(int i = 0; i < pts.size(); i++)
	{
		auto it = ref_ids_pts.find(ids[i]);
		if(it != ref_ids_pts.end())
		{
			cv::Point2f ref_kpts(it->second.x, it->second.y);
			cv::Point2f kpts(pts[i].x + ref_image.cols, pts[i].y);
			cv::circle(rgba_image, ref_kpts, 2, cv::Scalar(255, 0, 0), 2);
			cv::circle(rgba_image, kpts, 2, cv::Scalar(255, 0, 0), 2);
			//cv::line(rgba_image, ref_kpts, kpts, cv::Scalar(0,255,0, 10), 2);    
		}
	}
  	cv::cvtColor(rgba_image, imTrack, cv::COLOR_BGRA2BGR);
}

void FeatureTracker::readIntrinsicParameter()
{
	auto calib_file = feature_tracker_config.camera_config_file;
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);
    }
    if (calib_file.size() == 2)
        stereo_cam = true;
}

void FeatureTracker::readConfigParameter(const string &config_file, const string &model_prefix_path, const string &plugin_path)
{
	//feature_tracker config
	feature_tracker_config.load(config_file);
	readIntrinsicParameter();
	//plnet config
	PLNetConfig plnet_config;
	plnet_config.load(config_file);
	plnet_config.setModelPrefixPath(model_prefix_path);
	feature_detector = make_shared<FeatureDetector>(plnet_config);
	//point_match config
	PointMatcherConfig point_matcher_config;
	point_matcher_config.load(config_file);
	point_matcher_config.setModelPrefixPath(model_prefix_path);
	if(!plugin_path.empty())
		point_matcher_config.setPluginPath(plugin_path);
	point_matcher = make_shared<PointMatcher>(point_matcher_config);
	// //prewarm
	// feature_detector->prewarmInference();
	// point_matcher->prewarmInference();
}

void FeatureTracker::prewarmForTracker()
{
    int imgWidth = 640;
    int imgHeight = 480;
    int boardSize = 20;
    int cellWidth = imgWidth / boardSize;
    int cellHeight = imgHeight / boardSize;
    cv::Mat dummyImage0(imgHeight, imgWidth, CV_8UC1, cv::Scalar(0));
    for (int i = 0; i < boardSize; ++i) 
	{
        for (int j = 0; j < boardSize; ++j) 
		{
            int startX = i * cellWidth;
            int startY = j * cellHeight;
            if ((i + j) % 2 == 0) 
			{
				dummyImage0(cv::Rect(startX, startY, cellWidth, cellHeight)).setTo(cv::Scalar(255));
            }
        }
	}
	cv::Mat dummyImage1 = dummyImage0;
	//prewarm for superpoint
	TicToc tic_1;
	Eigen::Matrix<float, 259, Eigen::Dynamic> features0;
	Eigen::Matrix<float, 259, Eigen::Dynamic> features1;
	feature_detector->Detect(dummyImage0, features0);
	feature_detector->Detect(dummyImage1, features1);
	int dummy0PtsSize = features0.cols();
	vector<cv::Point2f> dummy_pts0;
	for(int i = 0; i < dummy0PtsSize; i++)
	{
		dummy_pts0.emplace_back(features0(1, i), features0(2, i));
	}
	ROS_DEBUG("prewarm superpoint cost %f ms, detect %d features.", tic_1.toc(), dummy0PtsSize);
	//prewarm for lightglue
	TicToc tic_2;
	vector<cv::DMatch> matches;
	point_matcher->MatchingPoints(features0, features1, matches, true);
	ROS_DEBUG("prewarm lightglue cost %f ms, matches size %d.", tic_2.toc(), matches.size());
	//prewarm for opticalflow
	TicToc tic_3;
	cv::cuda::GpuMat cur_gpu_img(dummyImage0);
	cv::cuda::GpuMat right_gpu_Img(dummyImage1);
	cv::cuda::GpuMat cur_gpu_pts(dummy_pts0);
	cv::cuda::GpuMat cur_right_gpu_pts;
	cv::cuda::GpuMat gpu_status;
	cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(
	cv::Size(21, 21), 3, 30, false);
	d_pyrLK_sparse->calc(cur_gpu_img, right_gpu_Img, cur_gpu_pts, cur_right_gpu_pts, gpu_status);
	ROS_DEBUG("prewarm opticalflow cost %f ms.", tic_3.toc());
	std::cout << "Prewarm for feature tracker completed!" << std::endl;
}

cv::Mat FeatureTracker::getTrackImage()
{
	return imTrack;
}

void FeatureTracker::calTrackCnt()
{
	cur_trackcnt_umap.clear();
	if(prev_trackcnt_umap.empty() && !cur_ids.empty())
	{
		for(auto id : cur_ids)
			cur_trackcnt_umap[id] = 1;
	}
	else
	{
		for(auto id : cur_ids)
		{
			auto prev_it = prev_trackcnt_umap.find(id);
			if(prev_it != prev_trackcnt_umap.end())
				cur_trackcnt_umap[id] = prev_it->second + 1;
			else
				cur_trackcnt_umap[id] = 1;
		}
	}
	prev_trackcnt_umap = cur_trackcnt_umap;
}
