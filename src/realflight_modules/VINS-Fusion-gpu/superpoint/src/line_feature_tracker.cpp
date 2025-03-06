#include "line_feature_tracker.h"
#include <numeric>
#include <algorithm>

static double distance(cv::Point2f pt1, cv::Point2f pt2)
{
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

inline double pixelToNormal(double pixel)
{
    return pixel / 389.6706237792969;
}

LineFeatureTracker::LineFeatureTracker()
{
    line_id = 0;
    prev_time = 0;
    cur_time = 0;
    curFrame.reset(new FrameLines);
    prevFrame.reset(new FrameLines);
}

void LineFeatureTracker::zAxisInCameraCallback(const sensor_msgs::PointCloudConstPtr &zc_msg)
{
    mtx_z.lock();
    cur_z[0] = zc_msg->points[0].x;
    cur_z[1] = zc_msg->points[0].y;
    cur_z[2] = zc_msg->points[0].z;
    cur_z.normalize();
    mtx_z.unlock();
    if(!is_z_usable)
        is_z_usable = true;
}

bool LineFeatureTracker::inBorder(const KeyLine &line)
{
    int BORDER_SIZE = line_tracker_config.borders;
    int start_x = cvRound(line.getStartPoint().x);
    int start_y = cvRound(line.getStartPoint().y);
    int end_x = cvRound(line.getEndPoint().x);
    int end_y = cvRound(line.getEndPoint().y);
    return (BORDER_SIZE <= start_x && start_x < line_tracker_config.col - BORDER_SIZE && BORDER_SIZE <= start_y && start_y < line_tracker_config.row - BORDER_SIZE) || 
           (BORDER_SIZE <= end_x && end_x < line_tracker_config.col - BORDER_SIZE && BORDER_SIZE <= end_y && end_y < line_tracker_config.row - BORDER_SIZE);
		
}
//返回z轴消失点在归一化坐标系中的坐标
cv::Point2f LineFeatureTracker::getVpzFromZc()
{
    mtx_z.lock();
    double scalar;
    if(fabs(cur_z.z()) < 1e-6)
        scalar = cur_z.z() < 0 ? -1e6 : 1e6;
    else
        scalar = 1 / cur_z.z();
    cv::Point2f vpz(cur_z.x(), cur_z.y());
    vpz *= scalar;
    mtx_z.unlock();
    return vpz;
}
//返回两条线的角度差(0~π/2)
double LineFeatureTracker::getTwoLinesAbsAngle(const KeyLine &line0, const KeyLine &line1)
{
    Vector2d l0_vec(line0.getEndPoint().x - line0.getStartPoint().x, line0.getEndPoint().y - line0.getStartPoint().y);
    Vector2d l1_vec(line1.getEndPoint().x - line1.getStartPoint().x, line1.getEndPoint().y - line1.getStartPoint().y);
    l0_vec.normalize();
    l1_vec.normalize();
    return acos(fabs(l0_vec.dot(l1_vec)));
}
//归一化坐标系下
double LineFeatureTracker::getTwoLinesAbsAngle(const Vector4d &line0, const Vector4d &line1)
{
    Vector2d l0_vec(line0[2] - line0[0], line0[3] - line0[1]);
    Vector2d l1_vec(line1[2] - line1[0], line1[3] - line1[1]);
    l0_vec.normalize();
    l1_vec.normalize();
    return acos(fabs(l0_vec.dot(l1_vec)));
}
//返回line1两个端点到直线line0的平均距离(像素系下)
double LineFeatureTracker::getTwoLinesDistByP2L(const KeyLine &line0, const KeyLine &line1)
{
    double x0 = line0.getStartPoint().x;
    double y0 = line0.getStartPoint().y;
    double x1 = line0.getEndPoint().x;
    double y1 = line0.getEndPoint().y;
    Vector3d l0(y1 - y0, x0 - x1, x1*y0 - x0*y1);
    double l0_norm = l0.head(2).norm();
    Vector3d l1_sp(line1.getStartPoint().x, line1.getStartPoint().y, 1.0);
    Vector3d l1_ep(line1.getEndPoint().x, line1.getEndPoint().y, 1.0);
    double dist_s = fabs(l0.dot(l1_sp) / l0_norm);
    double dist_e = fabs(l0.dot(l1_ep) / l0_norm);
    return (dist_s + dist_e) / 2;
}
//归一化坐标系下
double LineFeatureTracker::getTwoLinesDistByP2L(const Vector4d &line0, const Vector4d &line1)
{
    double x0 = line0[0];
    double y0 = line0[1];
    double x1 = line0[2];
    double y1 = line0[3];
    Vector3d l0(y1 - y0, x0 - x1, x1*y0 - x0*y1);
    double l0_norm = l0.head(2).norm();
    Vector3d l1_sp(line1[0], line1[1], 1.0);
    Vector3d l1_ep(line1[2], line1[3], 1.0);
    double dist_s = fabs(l0.dot(l1_sp) / l0_norm);
    double dist_e = fabs(l0.dot(l1_ep) / l0_norm);
    return (dist_s + dist_e) / 2;
}

//返回两个线段端点之间的距离
double LineFeatureTracker::getTwoLinesDistByP2P(const KeyLine &line0, const KeyLine &line1)
{
    cv::Point2f l0_sp = line0.getStartPoint();
    cv::Point2f l0_ep = line0.getEndPoint();
    cv::Point2f l1_sp = line1.getStartPoint();
    cv::Point2f l1_ep = line1.getEndPoint();
    if(distance(l0_sp, l1_sp) < distance(l0_sp, l1_ep))
        return (distance(l0_sp, l1_sp) + distance(l0_ep, l1_ep)) / 2;
    else
        return (distance(l0_sp, l1_ep) + distance(l0_ep, l1_sp)) / 2;
}
double LineFeatureTracker::getTwoLinesDistByMid(const KeyLine &line0, const KeyLine &line1)
{
    cv::Point2f l0_mid = (line0.getStartPoint() + line0.getEndPoint()) / 2;
    cv::Point2f l1_mid = (line1.getStartPoint() + line1.getEndPoint()) / 2;
    return distance(l0_mid, l1_mid);
}

Vector3d getLineExpression(const Vector4d &line)
{
    double x0 = line(0);
    double y0 = line(1);
    double x1 = line(2);
    double y1 = line(3);
    Vector3d l;
    l << y1 - y0, x0 - x1, x1*y0 - x0*y1;
    return l;
}

void LineFeatureTracker::readConfigParameter(const string &config_file)
{
    line_tracker_config.load(config_file);
	readIntrinsicParameter();
}

void LineFeatureTracker::readIntrinsicParameter()
{
    auto &calib_file = line_tracker_config.camera_config_file;
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[0]);
    K_ = m_camera->initUndistortRectifyMap(undist_map1,undist_map2);   
}

void LineFeatureTracker::undistortedLineEndPoints(const vector<KeyLine> &key_lsd, vector<Vector4d> &line_undist)
{
    line_undist.clear();
    line_undist.reserve(key_lsd.size());
    for(auto &line : key_lsd)
    {   
        Vector2d pts_s, pts_e;
        Vector3d un_pts_s, un_pts_e;
        pts_s << line.getStartPoint().x, line.getStartPoint().y;
        pts_e << line.getEndPoint().x, line.getEndPoint().y;
        m_camera->liftProjective(pts_s, un_pts_s);
        m_camera->liftProjective(pts_e, un_pts_e);
        un_pts_s /= un_pts_s.z();
        un_pts_e /= un_pts_e.z();
        line_undist.emplace_back(un_pts_s.x(), un_pts_s.y(), un_pts_e.x(), un_pts_e.y());
    }
}

void LineFeatureTracker::calCurTrackCnt()
{
    curFrame->trackCnt.clear();
    if(prevFrame->trackCnt.empty() && !curFrame->lineID.empty())
    {
        for(auto id : curFrame->lineID)
            curFrame->trackCnt[id] = 1;
    }
    else
    {
        for(auto id : curFrame->lineID)
        {
            auto it = prevFrame->trackCnt.find(id);
            if(it != prevFrame->trackCnt.end())
                curFrame->trackCnt[id] = it->second + 1;
            else
                curFrame->trackCnt[id] = 1;
        }
    }
}

void LineFeatureTracker::calCurVelocity()
{
    curFrame->un_id_linePts.clear();
    curFrame->lineVelocity.clear();
    for(int i = 0; i < curFrame->lineID.size(); i++)
    {
        curFrame->un_id_linePts[curFrame->lineID[i]] = curFrame->lineSpEpUndist[i];
    }

    if(!prevFrame->un_id_linePts.empty())
    {
        double dt = cur_time - prev_time;
        for(int i = 0; i < curFrame->lineID.size(); i++)
        {
            auto it = prevFrame->un_id_linePts.find(curFrame->lineID[i]);
            if(it != prevFrame->un_id_linePts.end())
            {
                Vector4d prev_se = it->second;
                Vector4d vel = (curFrame->lineSpEpUndist[i] - prev_se) / dt;
                curFrame->lineVelocity.push_back(vel);
            }
            else
                curFrame->lineVelocity.push_back(Vector4d(0, 0, 0, 0));
        }
    }
    else
    {
        for(int i = 0; i < curFrame->lineID.size(); i++)
            curFrame->lineVelocity.push_back(Vector4d(0, 0, 0, 0));
    }
}
//NMS实现
vector<int> LineFeatureTracker::lineNMSProcess(const vector<KeyLine> &vecTracked, const vector<KeyLine> &vecNew)
{
    vector<int> indexes(vecNew.size());
    iota(indexes.begin(), indexes.end(), 0);
    //remove lines that close to tracked
    int remove_cnt = 0;
    for(auto it = indexes.begin(); it != indexes.end();)
    {
        bool remove_it = false;
        int cur_id = *it;
        for(auto &k_line : vecTracked)
        {
            double diff_ang = getTwoLinesAbsAngle(k_line, vecNew[cur_id]);
            double diff_dist = getTwoLinesDistByP2P(k_line, vecNew[cur_id]);
            double diff_dist_mid = getTwoLinesDistByMid(k_line, vecNew[cur_id]);
            if(diff_ang < line_tracker_config.two_line_ang_thresh && (diff_dist < line_tracker_config.two_line_dist_thresh || diff_dist_mid < line_tracker_config.two_line_dist_thresh))
            {
                remove_it = true;
                remove_cnt++;
                break;
            }
        }
        if(remove_it)
            it = indexes.erase(it);
        else
            it++;
    }
    ROS_DEBUG("nms process-1 remove %d lines", remove_cnt);
    //sort by length
    sort(indexes.begin(), indexes.end(), [&vecNew](const int i1, const int i2)
        {
            return distance(vecNew[i1].getStartPoint(), vecNew[i1].getEndPoint()) > distance(vecNew[i2].getStartPoint(), vecNew[i2].getEndPoint());
        });
    //nms for new lines
    remove_cnt = 0;
    vector<int> res;
    while(!indexes.empty())
    {
        res.push_back(indexes.front());
        indexes.erase(indexes.begin());
        int cur_id = res.back();
        for(auto it = indexes.begin(); it != indexes.end();)
        {
            double diff_ang = getTwoLinesAbsAngle(vecNew[cur_id], vecNew[*it]);
            double diff_dist = getTwoLinesDistByP2P(vecNew[cur_id], vecNew[*it]);
            double diff_dist_mid = getTwoLinesDistByMid(vecNew[cur_id], vecNew[*it]);
            if(diff_ang < line_tracker_config.two_line_ang_thresh && (diff_dist < line_tracker_config.two_line_dist_thresh || diff_dist_mid < line_tracker_config.two_line_dist_thresh))
            {
                it = indexes.erase(it);
                remove_cnt++;
            }
            else
                it++;
        }
    }
    ROS_DEBUG("nms process-2 remove %d lines", remove_cnt);
    return res;
}
//划分出垂直线段
vector<LineType> LineFeatureTracker::lineClassify(const vector<Vector4d> &key_lsd, const cv::Point2f &vp_z)
{
    vector<LineType> type_res(key_lsd.size(), Hold);
    for(int i = 0; i < key_lsd.size(); i++)
    {
        double mid_x = (key_lsd[i][0] + key_lsd[i][2]) / 2;
        double mid_y = (key_lsd[i][1] + key_lsd[i][3]) / 2;
        Vector4d line_v(mid_x, mid_y, vp_z.x, vp_z.y);
        double diff_ang = getTwoLinesAbsAngle(line_v, key_lsd[i]);
        double diff_dist = getTwoLinesDistByP2L(line_v, key_lsd[i]);
        diff_dist *= 460.0;
        if(diff_ang < line_tracker_config.vertical_judge_ang && diff_dist < line_tracker_config.vertical_judge_dist)
            type_res[i] = VERTICAL;
    }
    return type_res;
}

void LineFeatureTracker::readImage(double _cur_time, const cv::Mat &_img)
{
    TicToc tic_all;
    if(line_tracker_config.detect_vertical_at_front)
    {
        if(!is_z_usable)
            return;
    }
    
    cur_time = _cur_time;
    cv::Mat img = _img.clone();
    //equalize
    if(line_tracker_config.equalize)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img, img);
    }

    if (first_image_flag) 
    {
        curFrame->img = img;
        prevFrame->img = img;
    }
    else
    {
        curFrame.reset(new FrameLines);
        curFrame->img = img;
    }

    //extract lsd
    TicToc t_li;
    Ptr<line_descriptor::LSDDetectorC> lsd_ = line_descriptor::LSDDetectorC::createLSDDetectorC();
    line_descriptor::LSDDetectorC::LSDOptions opts;
    opts.refine       = 1;     //1     	The way found lines will be refined
    opts.scale        = 0.5;   //0.8   	The scale of the image that will be used to find the lines. Range (0..1].
    opts.sigma_scale  = 0.6;	//0.6  	Sigma for Gaussian filter. It is computed as sigma = _sigma_scale/_scale.
    opts.quant        = 2.0;	//2.0   Bound to the quantization error on the gradient norm
    opts.ang_th       = 22.5;	//22.5	Gradient angle tolerance in degrees
    opts.log_eps      = 1.0;	//0		Detection threshold: -log10(NFA) > log_eps. Used only when advance refinement is chosen
    opts.density_th   = 0.6;	//0.7	Minimal density of aligned region points in the enclosing rectangle.
    opts.n_bins       = 1024;	//1024 	Number of bins in pseudo-ordering of gradient modulus.
    double min_line_length = 0.125;  // Line segments shorter than that are rejected
    opts.min_length   = min_line_length*(std::min(img.cols,img.rows));
    vector<KeyLine> lsd, keylsd;
    lsd_->detect(img, lsd, 2, 1, opts);
    ROS_DEBUG("line detect costs: %fms", t_li.toc());

    //extract desc
    TicToc t_lbd;
    Mat lbd_desc, keylbd_desc;
    Ptr<BinaryDescriptor> bd_ = BinaryDescriptor::createBinaryDescriptor();
    bd_->compute(img, lsd, lbd_desc);
    ROS_DEBUG("lbd extract cost %fms", t_lbd.toc());

    //extract keylsd
    for(int i = 0; i < lsd.size(); i++ )
    {
        if(lsd[i].octave == 0 && lsd[i].lineLength >= 60)
        {
            keylsd.push_back(lsd[i]);
            keylbd_desc.push_back(lbd_desc.row(i));
        }
    }
    curFrame->keyLsd = keylsd;
    curFrame->lbdDesc = keylbd_desc;
    curFrame->lineType = vector<LineType>(keylsd.size(), Hold);
    for (int i = 0; i < curFrame->keyLsd.size(); ++i) 
    {
        if(first_image_flag)
            curFrame->lineID.push_back(line_id++);
        else
            curFrame->lineID.push_back(-1);   // give a negative id
    }
    
    if(prevFrame->keyLsd.size() > 0)
    {
        //compute matches
        TicToc t_match;
        vector<DMatch> lsd_matches;
        Ptr<BinaryDescriptorMatcher> bdm_;
        bdm_ = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
        bdm_->match(curFrame->lbdDesc, prevFrame->lbdDesc, lsd_matches);
        ROS_DEBUG("lbd_macht costs: %fms", t_match.toc());
        //select best matches
        vector<DMatch> good_matches;
        vector<KeyLine> good_Keylines;
        good_matches.clear();
        for(int i = 0; i < lsd_matches.size(); i++)
        {
            if(lsd_matches[i].distance < 30)
            {
                DMatch mt = lsd_matches[i];
                KeyLine line1 =  curFrame->keyLsd[mt.queryIdx] ;
                KeyLine line2 =  prevFrame->keyLsd[mt.trainIdx] ;
                Point2f serr = line1.getStartPoint() - line2.getStartPoint();
                Point2f eerr = line1.getEndPoint() - line2.getEndPoint();
                if((serr.dot(serr) < 200 * 200) && (eerr.dot(eerr) < 200 * 200)&&abs(line1.angle-line2.angle)<0.1)   // 线段在图像里不会跑得特别远
                    good_matches.push_back(lsd_matches[i]);
            }
        }
        //assign id and type
        for(auto match : good_matches)
        {
            curFrame->lineID[match.queryIdx] = prevFrame->lineID[match.trainIdx];
            curFrame->lineType[match.queryIdx] = prevFrame->lineType[match.trainIdx];
        }

        //devide tracked and new
        vector<KeyLine> vecLineTracked, vecLineNew;
        vector<int> lineIdTracked, lineIdNew;
        Mat DescTracked, DescNew;
        vector<LineType> lineTypeTracked;
        for (size_t i = 0; i < curFrame->keyLsd.size(); ++i)
        {
            if( curFrame->lineID[i] == -1)
            {//new
                vecLineNew.push_back(curFrame->keyLsd[i]);
                lineIdNew.push_back(line_id++);
                DescNew.push_back(curFrame->lbdDesc.row(i));
            }
            else
            {
                vecLineTracked.push_back(curFrame->keyLsd[i]);
                lineIdTracked.push_back(curFrame->lineID[i]);
                DescTracked.push_back(curFrame->lbdDesc.row(i));
                lineTypeTracked.push_back(curFrame->lineType[i]);
            }
        }
        //NMS
        vector<int> nms_res;
        if(line_tracker_config.enable_line_nms)
        {
            nms_res = lineNMSProcess(vecLineTracked, vecLineNew);
            ROS_DEBUG("Before NMS new lines: %d, after NMS lines: %d, remove lines %d", vecLineNew.size(), nms_res.size(), vecLineNew.size() - nms_res.size());
        }
        else
        {
            nms_res = vector<int>(vecLineNew.size());
            iota(nms_res.begin(), nms_res.end(), 0);
        }
        //judge vertical for new line
        vector<KeyLine> vecLineNMS;
        vector<int> lineIdNMS;
        Mat DescNMS;
        vector<LineType> lineTypeNMS;
        for(auto id : nms_res)
        {
            vecLineNMS.push_back(vecLineNew[id]);
            lineIdNMS.push_back(lineIdNew[id]);
            DescNMS.push_back(DescNew.row(id));
        }
        if(line_tracker_config.detect_vertical_at_front)
        {
            vector<Vector4d> lineNMSUndist;
            undistortedLineEndPoints(vecLineNMS, lineNMSUndist);
            cv::Point2f vpz = getVpzFromZc();
            lineTypeNMS = lineClassify(lineNMSUndist, vpz);

            for(int i = 0; i < vecLineNMS.size(); i++)
            {
                vecLineTracked.push_back(vecLineNMS[i]);
                lineIdTracked.push_back(lineIdNMS[i]);
                DescTracked.push_back(DescNMS.row(i));
                lineTypeTracked.push_back(lineTypeNMS[i]);
            }
        }
        else
        {
            for(int i = 0; i < vecLineNMS.size(); i++)
            {
                vecLineTracked.push_back(vecLineNMS[i]);
                lineIdTracked.push_back(lineIdNMS[i]);
                DescTracked.push_back(DescNMS.row(i));
                lineTypeTracked.push_back(Hold);
            }
        }
        
        curFrame->keyLsd = vecLineTracked;
        curFrame->lineID = lineIdTracked;
        curFrame->lbdDesc = DescTracked;
        curFrame->lineType = lineTypeTracked;
    }

    //undistort
    undistortedLineEndPoints(curFrame->keyLsd, curFrame->lineSpEpUndist);
    //judge vertical at first image
    if(first_image_flag)
    {
        if(line_tracker_config.detect_vertical_at_front)
        {
            cv::Point2f vpz = getVpzFromZc();
            curFrame->lineType = lineClassify(curFrame->lineSpEpUndist, vpz);
        }
        else
        {
            curFrame->lineType = vector<LineType>(curFrame->keyLsd.size(), Hold);
        }
    }
    //calculate velocity
    calCurVelocity();
    //calculate track-cnts
    calCurTrackCnt();
    //draw line
    switch (line_tracker_config.show_line)
    {
    case 1:
        DrawLine();
        break;
    case 2:
        DrawLineWithType();
        break;
    default:
        break;
    }
    
    if(first_image_flag)
        first_image_flag = false;

    prev_time = cur_time;
    prevFrame = curFrame;
    ROS_INFO("line tracker cost %lf ms", tic_all.toc());
}

void LineFeatureTracker::DrawLine()
{
   	cv::Mat rgba_image;
	cv::cvtColor(curFrame->img, rgba_image, cv::COLOR_BGR2BGRA);
	for(int i = 0; i < curFrame->lineID.size(); i++)
	{
		int track_cnts;
		auto it = curFrame->trackCnt.find(curFrame->lineID[i]);
		if(it == curFrame->trackCnt.end())
			track_cnts = 0;
		else
			track_cnts = it->second;
		double len = std::min(1.0, 1.0 * track_cnts / 20);
        cv::line(rgba_image, curFrame->keyLsd[i].getStartPoint(), curFrame->keyLsd[i].getEndPoint(), cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
	}
	cv::cvtColor(rgba_image, imTrack, cv::COLOR_BGRA2BGR); 
}

void LineFeatureTracker::DrawLineWithType()
{
   	cv::Mat rgba_image;
	cv::cvtColor(curFrame->img, rgba_image, cv::COLOR_BGR2BGRA);
    //draw line
    for(int i = 0; i < curFrame->lineID.size(); i++)
    {
        if(curFrame->lineType[i] == VERTICAL)
            cv::line(rgba_image, curFrame->keyLsd[i].getStartPoint(), curFrame->keyLsd[i].getEndPoint(), cv::Scalar(255, 0, 0), 2);//BGR
        else
            cv::line(rgba_image, curFrame->keyLsd[i].getStartPoint(), curFrame->keyLsd[i].getEndPoint(), cv::Scalar(0, 255, 0), 2);
    }

    cv::cvtColor(rgba_image, imTrack, cv::COLOR_BGRA2BGR); 
}

void LineFeatureTracker::DrawLIneWithAssociaPts()
{
    cv::Mat rgba_image;
	cv::cvtColor(curFrame->img, rgba_image, cv::COLOR_BGR2BGRA);
    for(int i = 0; i < curFrame->lineID.size(); i++)
    {
        cv::line(rgba_image, curFrame->keyLsd[i].getStartPoint(), curFrame->keyLsd[i].getEndPoint(), cv::Scalar(255, 0, 0), 2);
        for(int j = 0; j < curFrame->lineAssociaPts[i].size(); j++)
        {
            cv::circle(rgba_image, curFrame->lineAssociaPts[i][j], 2, cv::Scalar(0, 0, 255), 2);
        }
    }
    cv::cvtColor(rgba_image, imTrack, cv::COLOR_BGRA2BGR); 
}

cv::Mat LineFeatureTracker::getTrackImage()
{
	return imTrack;
}

// 3 / fx = 0.00769
void LineFeatureTracker::calAssociaPtsForLines(const vector<cv::Point2f> &cur_un_pts, const vector<cv::Point2f> &cur_pts, const vector<Vector4d> &cur_un_lines, vector<vector<cv::Point2f>> &associa_pts)
{
    assert(cur_un_pts.size() == cur_pts.size());
    int idx = 0; 
    MatrixXd cur_pts_mat(cur_un_pts.size(), 3);
    for(auto &un_pt : cur_un_pts)
    {
        Vector3d pt;
        pt << un_pt.x, un_pt.y, 1.0;
        cur_pts_mat.row(idx++) = pt;
    }
    int line_idx = 0;
    associa_pts.clear();
    for(auto &line : cur_un_lines)
    {
        double lx1 = line(0);
        double ly1 = line(1);
        double lx2 = line(2);
        double ly2 = line(3);
        double min_lx = lx1;
        double max_lx = lx2;
        double min_ly = ly1;
        double max_ly = ly2;
        if(lx1 > lx2) std::swap(min_lx, max_lx);
        if(ly1 > ly2) std::swap(min_ly, max_ly);
        //cal distance
        Vector3d l_param = getLineExpression(line);
        double l_norm = l_param.head(2).norm();
        VectorXd pts_l_dist = cur_pts_mat * l_param / l_norm;
        pts_l_dist = pts_l_dist.cwiseAbs();
        //associate pts
        int cnt = 0;
        vector<cv::Point2f> line_associa_pts;
        for(int i = 0; i < pts_l_dist.rows(); i++)
        {
            //ROS_DEBUG("calAssociaPtsForLines: cur pts is (%lf, %lf), line is (%lf, %lf, %lf, %lf), pts_l_dist is %lf", cur_pts_mat(i, 0), cur_pts_mat(i, 1), line(0), line(1), line(2), line(3), pts_l_dist(i));
            if(pts_l_dist(i) > pixelToNormal(3))
                continue;

            double px = cur_pts_mat(i, 0);
            double py = cur_pts_mat(i, 1);
            if(px < min_lx - pixelToNormal(3) || px > max_lx + pixelToNormal(3) || py < min_ly - pixelToNormal(3) || py > max_ly + pixelToNormal(3)) continue;

            double side1 = std::pow((lx1 - px), 2) + std::pow((ly1 - py), 2);
            double side2 = std::pow((lx2 - px), 2) + std::pow((ly2 - py), 2);
            double line_side = std::pow(l_norm, 2);
            if(side1 <= pixelToNormal(9) || side2 <= pixelToNormal(9) || ((side1 < line_side + side2) && (side2 < line_side + side1)))
            {
                line_associa_pts.push_back(cur_pts[i]);
                cnt++;
            }
        }
        associa_pts.push_back(line_associa_pts);
        //ROS_DEBUG("calAssociaPtsForLines: line-%d was associated %d points", line_idx, cnt);
        line_idx++;
    }
}
