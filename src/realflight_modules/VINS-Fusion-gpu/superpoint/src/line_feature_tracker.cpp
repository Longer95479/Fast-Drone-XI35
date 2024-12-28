#include "line_feature_tracker.h"

LineFeatureTracker::LineFeatureTracker()
{
    line_id = 0;
    prev_time = 0;
    cur_time = 0;
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

void LineFeatureTracker::readImage(double _cur_time, const cv::Mat &_img)
{
    cur_time = _cur_time;
    cv::Mat img = _img;
    //equalize
    if(line_tracker_config.equalize)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img, img);
    }

    if (first_image_flag) 
    {
        curFrame.reset(new FrameLines);
        prevFrame.reset(new FrameLines);
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
    for (int i = 0; i < curFrame->keyLsd.size(); ++i) 
    {
        if(first_image_flag)
            curFrame->lineID.push_back(line_id++);
        else
            curFrame->lineID.push_back(-1);   // give a negative id
    }
    if(first_image_flag)
        first_image_flag = false;
    
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
        //assign id
        for(auto match : good_matches)
            curFrame->lineID[match.queryIdx] = prevFrame->lineID[match.trainIdx];

        //devide tracked and new
        vector<KeyLine> vecLineTracked, vecLineNew;
        vector<int> lineIdTracked, lineIdNew;
        Mat DescTracked, DescNew;
        for (size_t i = 0; i < curFrame->keyLsd.size(); ++i)
        {
            if( curFrame->lineID[i] == -1)
            {//new
                vecLineNew.push_back(curFrame->keyLsd[i]);
                lineIdNew.push_back(line_id++);
                DescNew.push_back( curFrame->lbdDesc.row(i));
            }
            else
            {
                vecLineTracked.push_back(curFrame->keyLsd[i]);
                lineIdTracked.push_back(curFrame->lineID[i]);
                DescTracked.push_back( curFrame->lbdDesc.row(i));
            }
        }
        //devide h and v in new lines
        vector<KeyLine> h_line_new, v_line_new;
        vector< int > h_lineId_new,v_lineId_new;
        Mat h_desc_new,v_desc_new;
        for (size_t i = 0; i < vecLineNew.size(); ++i)
        {
            if((((vecLineNew[i].angle >= 3.14/4 && vecLineNew[i].angle <= 3*3.14/4))||(vecLineNew[i].angle <= -3.14/4 && vecLineNew[i].angle >= -3*3.14/4)))
            {
                h_line_new.push_back(vecLineNew[i]);
                h_lineId_new.push_back(lineIdNew[i]);
                h_desc_new.push_back(DescNew.row( i ));
            }
            else
            {
                v_line_new.push_back(vecLineNew[i]);
                v_lineId_new.push_back(lineIdNew[i]);
                v_desc_new.push_back(DescNew.row( i ));
            }      
        }
        int h_line,v_line;
        h_line = v_line =0;
        for (size_t i = 0; i < vecLineTracked.size(); ++i)
        {
            if((((vecLineTracked[i].angle >= 3.14/4 && vecLineTracked[i].angle <= 3*3.14/4))||(vecLineTracked[i].angle <= -3.14/4 && vecLineTracked[i].angle >= -3*3.14/4)))
            {
                h_line ++;
            }
            else
            {
                v_line ++;
            }
        }
        int diff_h = 35 - h_line;
        int diff_v = 35 - v_line;
        //add lines
        if(diff_h > 0)
        {
            if(diff_h > h_line_new.size())
                diff_h = h_line_new.size();
            for (int k = 0; k < diff_h; ++k) 
            {
                vecLineTracked.push_back(h_line_new[k]);
                lineIdTracked.push_back(h_lineId_new[k]);
                DescTracked.push_back(h_desc_new.row(k));
            }
        }
        if(diff_v > 0)    // 补充线条
        {
            if(diff_v > v_line_new.size())
                diff_v = v_line_new.size();
            for (int k = 0; k < diff_v; ++k)  
            {
                vecLineTracked.push_back(v_line_new[k]);
                lineIdTracked.push_back(v_lineId_new[k]);
                DescTracked.push_back(v_desc_new.row(k));
            }  
        }

        curFrame->keyLsd = vecLineTracked;
        curFrame->lineID = lineIdTracked;
        curFrame->lbdDesc = DescTracked;
    }

    //undistort
    undistortedLineEndPoints(curFrame->keyLsd, curFrame->lineSpEpUndist);
    //calculate velocity
    calCurVelocity();
    //calculate track-cnts
    calCurTrackCnt();
    //draw line
    DrawLine();

    prev_time = cur_time;
    prevFrame = curFrame;
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

cv::Mat LineFeatureTracker::getTrackImage()
{
	return imTrack;
}
