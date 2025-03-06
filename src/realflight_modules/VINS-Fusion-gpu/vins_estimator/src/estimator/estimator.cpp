/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"
#include "../utility/visualization.h"
#include <fstream>
#include <algorithm>

inline double pixelToNormal(double pixel)
{
    return pixel / 389.6706237792969;
}

inline double normalToPixel(double normal)
{
    return normal * 389.6706237792969;
}

Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    clearState();
    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    // sum_t_feature = 0.0;
    // begin_time_count = 10;
    initFirstPoseFlag = false;
    temp_cur_time = 0;
    temp_last_time = 0;
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        cout << " exitrinsic cam " << i << endl  << ric[i] << endl << tic[i].transpose() << endl;
    }
    f_manager.setRic(ric);
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    LineProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    StructLineProjectionOneFrameFactor::sqrt_info = FOCAL_LENGTH / 1.5 * STRUCT_LINE_SQRT_INFO * Matrix2d::Identity();
    StructLineProjectionTwoFrameFactor::sqrt_info = FOCAL_LENGTH / 1.5 * STRUCT_LINE_SQRT_INFO * Matrix2d::Identity();
    HorizonLineProjectionOneFrameFactor::sqrt_info = FOCAL_LENGTH / 1.5 * STRUCT_LINE_SQRT_INFO * Matrix2d::Identity();
    HorizonLineProjectionTwoFrameFactor::sqrt_info = FOCAL_LENGTH / 1.5 * STRUCT_LINE_SQRT_INFO * Matrix2d::Identity();
    VerticalLineProjectionOneFrameFactor::sqrt_info = FOCAL_LENGTH / 1.5 * STRUCT_LINE_SQRT_INFO * Matrix2d::Identity();
    VerticalLineProjectionTwoFrameFactor::sqrt_info = FOCAL_LENGTH / 1.5 * STRUCT_LINE_SQRT_INFO * Matrix2d::Identity();

    td = TD;
    g = G;
    cout << "set g " << g.transpose() << endl;
    featureTracker.readIntrinsicParameter(CAM_NAMES);

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD)
    {
        processThread   = std::thread(&Estimator::processMeasurements, this);
    }
}

void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
{
//     if(begin_time_count<=0)
    inputImageCnt++;
    pair< map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > >, map<int, Eigen::Matrix<double, 8, 1> > >  featureFrame;
    TicToc featureTrackerTime;
    if(_img1.empty())
        featureFrame.first = featureTracker.trackImage(t, _img);
    else
        featureFrame.first = featureTracker.trackImage(t, _img, _img1);
    // if(begin_time_count--<=0)
    // {
    //     sum_t_feature += featureTrackerTime.toc();
    //     printf("featureTracker time: %f\n", sum_t_feature/(float)inputImageCnt);
    // }
    //ROS_INFO("Track cost %f ms.", featureTrackerTime.toc());
    if (SHOW_TRACK)
    {
        cv::Mat imgTrack = featureTracker.getTrackImage();
        pubTrackImage(imgTrack, t);
    }
    
    if(MULTIPLE_THREAD)  
    {     
        if(inputImageCnt % 2 == 0)
        {
            mBuf.lock();
            featureBuf.push(make_pair(t, featureFrame));
            mBuf.unlock();
        }
    }
    else
    {
        mBuf.lock();
        featureBuf.push(make_pair(t, featureFrame));
        mBuf.unlock();
        TicToc processTime;
        processMeasurements();
        printf("process time: %f\n", processTime.toc());
    }
    
}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    mBuf.lock();
    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
    //printf("input imu with time %f \n", t);
    mBuf.unlock();

    fastPredictIMU(t, linearAcceleration, angularVelocity);
    if (solver_flag == NON_LINEAR)
    {
        pubLatestOdometry(latest_P, latest_Q, latest_V, t);
        pubWorldZinCamera(latest_Q, ric[0], t);
    }
}

void Estimator::inputFeature(double t, const pair< map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > >, map<int, Eigen::Matrix<double, 8, 1> > > &featureFrame)
{
    mBuf.lock();
    featureBuf.push(make_pair(t, featureFrame));
    mBuf.unlock();
    if(!MULTIPLE_THREAD)
        processMeasurements();
}


bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if(accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    //printf("get imu from %f %f\n", t0, t1);
    //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if(t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}

bool Estimator::IMUAvailable(double t)
{
    if(!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}

void Estimator::processMeasurements()
{
    while (1)
    {
        //printf("process measurments\n");
        TicToc t_process;
        pair<double, pair<map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > >, map<int, Eigen::Matrix<double, 8, 1> > > > feature;
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
        if(!featureBuf.empty())
        {
            feature = featureBuf.front();
            curTime = feature.first + td;
            while(1)
            {
                if ((!USE_IMU  || IMUAvailable(feature.first + td)))
                    break;
                else
                {
                    printf("wait for imu ... ,td is %lf\n", td);
                    if (! MULTIPLE_THREAD)
                        return;
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
            }
            mBuf.lock();
            if(USE_IMU)
                getIMUInterval(prevTime, curTime, accVector, gyrVector);

            featureBuf.pop();
            mBuf.unlock();

            if(USE_IMU)
            {
                if(!initFirstPoseFlag)
                    initFirstIMUPose(accVector);
                for(size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;
                    if(i == 0)
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                }
            }
            if(USE_STRUCT_LINE)
                processImageWithPointsAndStructLines(feature.second, feature.first);
            else
                processImage(feature.second, feature.first);
            prevTime = curTime;

            printStatistics(*this, 0);

            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time(feature.first);

            //publish image
            if(PUB_IMAGE_AT_BACKEND)
            {
                DrawImage(feature.first);
                pubBackendImage(*this, header);
            }

            pubOdometry(*this, header);
            pubKeyPoses(*this, header);
            pubCameraPose(*this, header);
            pubPointCloud(*this, header);
            if(USE_STRUCT_LINE)
                pubStructLinesCloud(*this, header);
            else
                pubLinesCloud(*this, header);
            pubKeyframe(*this);
            pubTF(*this, header);
            printf("current used features counts: %d.\n", f_manager.getFeatureCount());
            printf("process measurement time: %f\n", t_process.toc());
            if(record_csv)
            {
                double cur_timestamp = feature.first;
                if(!csv_file_path.empty())
                {
                    std::ofstream ofs;
                    ofs.open(csv_file_path, std::ios_base::app);
                    if(ofs.is_open())
                    {
                        ofs << cur_timestamp << " ";
                        ofs << f_manager.getFeatureCount() << " ";
                        ofs << Ps[WINDOW_SIZE].x() << " ";
                        ofs << Ps[WINDOW_SIZE].y() << " ";
                        ofs << Ps[WINDOW_SIZE].z() << " ";
                        ofs << cur_removed_counts << " ";
                        ofs << temp_cur_V_norm << " ";
                        ofs << td << "\n";
                        ofs.close();
                    }
                }
            }
        }

        if (! MULTIPLE_THREAD)
            break;

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    //return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for(size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
    cout << "init R0 " << endl << Rs[0] << endl;
    //Vs[0] = Vector3d(5, 0, 0);
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r)
{
    Ps[0] = p;
    Rs[0] = r;
    initP = p;
    initR = r;
}


void Estimator::clearState()
{
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    local_mht = 0;
    mht_state = UPDATING;
    initial_timestamp = 0;
    all_image_frame.clear();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;
}

void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity; 
}

void Estimator::processImage(const pair<map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > >, map<int, Eigen::Matrix<double, 8, 1> > > &image, const double header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.first.size());
    ROS_DEBUG("Adding line features %lu", image.second.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image.first, td))
    {
        marginalization_flag = MARGIN_OLD;
        //printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        //printf("non-keyframe\n");
    }
    
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image.first, header);
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    //add line
    if(!image.second.empty())
    {
        line_manager.addLineFeature(frame_count, image.second, td);
    }

    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        // monocular + IMU initilization
        if (!STEREO && USE_IMU)
        {
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;
                if(ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    result = initialStructure();
                    initial_timestamp = header;   
                }
                if(result)
                {
                    solver_flag = NON_LINEAR;
                    optimization();
                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
                else
                    slideWindow();
            }
        }

        // stereo + IMU initilization
        if(STEREO && USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            line_manager.line_triangulate(Rs, Ps, tic, ric);
            if (frame_count == WINDOW_SIZE)
            {
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }
                solveGyroscopeBias(all_image_frame, Bgs);
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
                }
                solver_flag = NON_LINEAR;
                onlyLinesOptimization();
                optimization();
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        // stereo only initilization
        if(STEREO && !USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            optimization();

            if(frame_count == WINDOW_SIZE)
            {
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        if(frame_count < WINDOW_SIZE)
        {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }

    }
    else
    {
        TicToc t_solve;
        if(!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        //points and line triangulate
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        line_manager.line_triangulate(Rs, Ps, tic, ric);
        //optimization
        if(enable_triang_opti_only)
            onlyLinesOptimization();
        optimization();
        //remove points outliers
        set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);
        cur_removed_counts = removeIndex.size();
        if (! MULTIPLE_THREAD)
        {
            featureTracker.removeOutliers(removeIndex);
            predictPtsInNextFrame();
        }
        //remove line outliers
        removeIndex.clear();
        lineOutliersRejection(removeIndex);
        line_manager.removeOutlier(removeIndex);
        ROS_DEBUG("line remove outliers counts: %d", removeIndex.size());
            
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        slideWindow();
        f_manager.removeFailures();
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        if(enable_imu_odom_smooth)
        {
            calCurVelocity(header, Ps[WINDOW_SIZE]);
            if(temp_cur_V_norm > velocity_limit && !have_dropped_one_frame)
            {
                ROS_WARN("Curretn velocity exceed limitation! would not update the latest states.");
                have_dropped_one_frame = true;
            }
            else
            {
                updateLatestStates();
                have_dropped_one_frame = false;
            }
        }
        else
            updateLatestStates();
    }  
}
void Estimator::processImageWithPointsAndStructLines(const pair<map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > >, map<int, Eigen::Matrix<double, 8, 1> > > &image, const double header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.first.size());
    ROS_DEBUG("Adding line features %lu", image.second.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image.first, td))
    {
        marginalization_flag = MARGIN_OLD;
        //printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        //printf("non-keyframe\n");
    }
    
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image.first, header);
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        // monocular + IMU initilization
        if (!STEREO && USE_IMU)
        {
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;
                if(ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    result = initialStructure();
                    initial_timestamp = header;   
                }
                if(result)
                {
                    solver_flag = NON_LINEAR;
                    optimization();
                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
                else
                    slideWindow();
            }
        }

        // stereo + IMU initilization
        if(STEREO && USE_IMU)
        {
            //point process
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            //struct line process
            //add lines
            vector<pair<int, Eigen::Matrix<double, 8, 1>>> tracked_h_lines, new_lines;
            struct_line_manager.addTrackedStructLineAndGetHorizon(image.second, td, tracked_h_lines, new_lines);
            //vertical line triangulate
            struct_line_manager.onlyVerticalLineTriangulate(Rs, Ps, tic, ric);
            if(ASSOCIATE_POINTS_TO_LINES)
            {
                struct_line_manager.onlyVerticalLineTriangulateByPoints(f_manager, Rs, Ps, tic, ric);
            }
            //classify the vertical lines among new lines
            vector<pair<int, Eigen::Matrix<double, 8, 1>>> new_vertical_lines, new_other_lines;
            onlyClassifyVerticalLine(new_lines, new_vertical_lines, new_other_lines);
            int new_other_size = new_other_lines.size();
            //RANSAC
            new_other_lines.insert(new_other_lines.end(), tracked_h_lines.begin(), tracked_h_lines.end());
            auto res_ransac = recognizeMHTUsingRANSAC(frame_count, new_other_lines);
            double new_mht = res_ransac.first? res_ransac.second : mht_manager.getLatestMHT();
            //classify the horizon lines among new lines
            vector<pair<int, Eigen::Matrix<double, 8, 1>>> new_horizon_lines;
            vector<LineType> new_horizon_lines_type;
            new_other_lines.resize(new_other_size);//only classify the new lines
            onlyClassifyHorizonLine(new_mht, new_other_lines, new_horizon_lines, new_horizon_lines_type);
            //add to line manager
            vector<LineType> vertical_type(new_vertical_lines.size(), VERTICAL);
            struct_line_manager.addNewStrcutLine(frame_count, new_vertical_lines, vertical_type, td);
            struct_line_manager.addNewStrcutLine(frame_count, new_horizon_lines, new_horizon_lines_type, td);
            //add to mht manager
            mht_manager.insertNewMHT(frame_count, new_mht);
            mht_manager.printMHTWindow();
            //assocaite points to lines
            if(ASSOCIATE_POINTS_TO_LINES)
            {
                vector<pair<int, Vector4d>> lines_n_trig;
                struct_line_manager.getUninitialLines(image.second, lines_n_trig);
                vector<pair<int, vector<pair<int, double>>>> lines_associa_pts;
                calAssociaPtsForLines(image.first, lines_n_trig, lines_associa_pts);
                struct_line_manager.updateLinesAssociaPts(lines_associa_pts);
            }

            if (frame_count == WINDOW_SIZE)
            {
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }
                solveGyroscopeBias(all_image_frame, Bgs);
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
                }
                solver_flag = NON_LINEAR;
                //check to merge the local mht
                if(mht_manager.checkMHTWindow())
                {
                    //update local_mht
                    local_mht = mht_manager.getMeanMHT();
                    //Triangulate the new lines
                    struct_line_manager.structLineTriangulate(local_mht, Rs, Ps, tic, ric);
                    if(ASSOCIATE_POINTS_TO_LINES)
                    {
                        struct_line_manager.structLineTriangulateByPoints(local_mht, f_manager, Rs, Ps, tic, ric);
                    }
                    //only optimize the local mht and lines
                    onlyOptimizeMhtAndLines();
                    mht_state = HOLD;
                    mht_manager.printMHTWindow();
                    ROS_INFO("local mht initialization finish!");
                }
                optimization();
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        // stereo only initilization
        if(STEREO && !USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            optimization();

            if(frame_count == WINDOW_SIZE)
            {
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        if(frame_count < WINDOW_SIZE)
        {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }

    }
    else
    {
        TicToc t_solve;
        if(!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);

        //points triangulate
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

        //line process
        if(mht_state == HOLD)
        {
            //add lines
            vector<pair<int, Eigen::Matrix<double, 8, 1>>> new_lines;
            struct_line_manager.addTrackedStructLine(image.second, td, new_lines);

            //line triangulate
            TicToc tic_tri;
            struct_line_manager.structLineTriangulate(local_mht, Rs, Ps, tic, ric);
            if(ASSOCIATE_POINTS_TO_LINES)
            {
                struct_line_manager.structLineTriangulateByPoints(local_mht, f_manager, Rs, Ps, tic, ric);
            }
            ROS_DEBUG("Hold: structLineTriangulate cost %lf ms.", tic_tri.toc());

            //optimization
            TicToc tic_opt;
            double sline_err_bf = calAllStructLinesReprojectionError();
            optimization();
            double sline_err_af = calAllStructLinesReprojectionError();
            ROS_DEBUG("Strut line rep err changing after optimization: %lf ===> %lf", sline_err_bf, sline_err_af);
            ROS_DEBUG("Hold: optimization cost %lf ms.", tic_opt.toc());

            //classify the new lines
            TicToc tic_cn;
            vector<pair<int, Eigen::Matrix<double, 8, 1>>> add_lines;
            vector<LineType> add_lines_type;
            bool trigger_mht_detect = structLineClassify(new_lines, add_lines, add_lines_type);
            ROS_DEBUG("Hold: structLineClassify cost %lf ms.", tic_cn.toc());
            if(trigger_mht_detect)
            {
                mht_state = UPDATING;
                mht_manager.clear();
                ROS_WARN("Trigger new mht detection! The old local mht is %lf", local_mht);
            }
            //add to manager
            struct_line_manager.addNewStrcutLine(frame_count, add_lines, add_lines_type, td);
        }
        else
        {//updating local mht
            //add lines
            vector<pair<int, Eigen::Matrix<double, 8, 1>>> tracked_h_lines, new_lines;
            struct_line_manager.addTrackedStructLineAndGetHorizon(image.second, td, tracked_h_lines, new_lines);
            //line triangulate
            struct_line_manager.onlyVerticalLineTriangulate(Rs, Ps, tic, ric);
            if(ASSOCIATE_POINTS_TO_LINES)
            {
                struct_line_manager.onlyVerticalLineTriangulateByPoints(f_manager, Rs, Ps, tic, ric);
            }
            //optimization
            optimization();
            //classify the vertical lines among new lines
            vector<pair<int, Eigen::Matrix<double, 8, 1>>> new_vertical_lines, new_other_lines;
            onlyClassifyVerticalLine(new_lines, new_vertical_lines, new_other_lines);
            int new_other_size = new_other_lines.size();
            //RANSAC
            TicToc tic_rac;
            new_other_lines.insert(new_other_lines.end(), tracked_h_lines.begin(), tracked_h_lines.end());
            auto res_ransac = recognizeMHTUsingRANSAC(frame_count, new_other_lines);
            double new_mht = res_ransac.first? res_ransac.second : mht_manager.getLatestMHT();
            ROS_DEBUG("Updating: RANSAC cost %lf ms.", tic_rac.toc());
            //classify the horizon lines among new lines
            vector<pair<int, Eigen::Matrix<double, 8, 1>>> new_horizon_lines;
            vector<LineType> new_horizon_lines_type;
            new_other_lines.resize(new_other_size);//only classify the new lines
            onlyClassifyHorizonLine(new_mht, new_other_lines, new_horizon_lines, new_horizon_lines_type);
            //add to line manager
            vector<LineType> vertical_type(new_vertical_lines.size(), VERTICAL);
            struct_line_manager.addNewStrcutLine(frame_count, new_vertical_lines, vertical_type, td);
            struct_line_manager.addNewStrcutLine(frame_count, new_horizon_lines, new_horizon_lines_type, td);
            //add to mht manager
            mht_manager.insertNewMHT(frame_count, new_mht);
            mht_manager.printMHTWindow();
            //check to merge the local mht
            if(mht_manager.checkMHTWindow())
            {
                //update local_mht
                local_mht = mht_manager.getMeanMHT();
                //Triangulate the new lines
                struct_line_manager.structLineTriangulate(local_mht, Rs, Ps, tic, ric);
                if(ASSOCIATE_POINTS_TO_LINES)
                {
                    struct_line_manager.structLineTriangulateByPoints(local_mht, f_manager, Rs, Ps, tic, ric);
                }
                //only optimize the local mht and lines
                onlyOptimizeMhtAndLines();
                mht_state = HOLD;
                ROS_INFO("Local mht updating finish! The new local mht is %lf", local_mht);
            }
        }
        //assocaite points to lines
        if(ASSOCIATE_POINTS_TO_LINES)
        {
            TicToc tic_ap;
            vector<pair<int, Vector4d>> lines_n_trig;
            struct_line_manager.getUninitialLines(image.second, lines_n_trig);
            vector<pair<int, vector<pair<int, double>>>> lines_associa_pts;
            calAssociaPtsForLines(image.first, lines_n_trig, lines_associa_pts);
            struct_line_manager.updateLinesAssociaPts(lines_associa_pts);
            ROS_DEBUG("Associate points to lines cost %lf ms.", tic_ap.toc());
        }

        auto line_cnt = struct_line_manager.getTriangulatedCount();
        ROS_DEBUG("Struct line manager has %d line features, %d is triangulated.", line_cnt.first, line_cnt.second);
        
        //remove points outliers
        set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);
        cur_removed_counts = removeIndex.size();
        if (! MULTIPLE_THREAD)
        {
            featureTracker.removeOutliers(removeIndex);
            predictPtsInNextFrame();
        }
        //remove line outliers
        removeIndex.clear();
        structLineOutliersRejection(removeIndex);
        auto rm_cnt = struct_line_manager.removeOutlier(removeIndex);
        ROS_DEBUG("struct line remove outliers counts: %d, remove %d vertical lines, %d horizon lines.", removeIndex.size(), rm_cnt.first, rm_cnt.second);
            
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        slideWindow();
        f_manager.removeFailures();
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];

        if(enable_imu_odom_smooth)
        {
            calCurVelocity(header, Ps[WINDOW_SIZE]);
            if(temp_cur_V_norm > velocity_limit && !have_dropped_one_frame)
            {
                ROS_WARN("Curretn velocity exceed limitation! would not update the latest states.");
                have_dropped_one_frame = true;
            }
            else
            {
                updateLatestStates();
                have_dropped_one_frame = false;
            }
        }
        else
            updateLatestStates();
    }  
}
bool Estimator::initialStructure()
{
    TicToc t_sfm;
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            //return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }

}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose()); 

    f_manager.clearDepth();
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

    return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if(USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }


    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    para_Td[0][0] = td;
    
    //line
    if(USE_STRUCT_LINE)
    {
        MatrixXd sline_mat = struct_line_manager.getLineParamMat(struct_lines_opt_type);
        for(int i = 0; i < struct_line_manager.getFeatureCount(); i++)
        {
            para_Struct_Line[i][0] = sline_mat.row(i)[0];
            para_Struct_Line[i][1] = sline_mat.row(i)[1];
        }
        para_Local_MHT[0][0] = local_mht;
    }
    else if(enable_line_opti || enable_triang_opti_only)
    {
        MatrixXd line_orth_mat = line_manager.getLineOrthMat();
        for(int i = 0; i < line_manager.getFeatureCount(); i++)
        {
            para_Line[i][0] = line_orth_mat.row(i)[0];
            para_Line[i][1] = line_orth_mat.row(i)[1];
            para_Line[i][2] = line_orth_mat.row(i)[2];
            para_Line[i][3] = line_orth_mat.row(i)[3];
        }
    }
}

void Estimator::double2vector()
{
    // 六自由度优化的时候，整个窗口会在空间中任意优化，这时候我们需要把第一帧在yaw,position上的增量给去掉，因为vins在这几个方向上不可观，他们优化的增量也不可信。
    // 所以这里的操作过程就相当于是 fix 第一帧的 yaw 和 postion, 使得整个轨迹不会在空间中任意飘。
    // 也可以理解为世界坐标系在yaw和position方向上发生了偏移，需要矫正回去。矫正方法：建立新的世界坐标系，滑窗中的第一帧在新世界系下的位姿是优化前的位姿。然后把滑窗中的所有状态都转到新世界系下
    // 相机姿态需要变化考虑优化以后，把yaw量旋转回去
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    if(USE_IMU)
    {
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                          para_Pose[0][3],
                                                          para_Pose[0][4],
                                                          para_Pose[0][5]).toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
        double y_diff_rad = y_diff / 180.0 * M_PI;
        ROS_DEBUG("Current y_diff_rad is %lf", y_diff_rad);
        //TODO
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5]).toRotationMatrix().transpose();
        }

        Vector3d ps0(para_Pose[0][0],para_Pose[0][1],para_Pose[0][2]);
        Matrix3d Rwn_wo(rot_diff);//旧世界系到新世界系的旋转
        Vector3d twn_wo = -Rwn_wo * ps0 + origin_P0;//旧世界系到新世界系的位移

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {

            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;


                Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                            para_SpeedBias[i][1],
                                            para_SpeedBias[i][2]);

                Bas[i] = Vector3d(para_SpeedBias[i][3],
                                  para_SpeedBias[i][4],
                                  para_SpeedBias[i][5]);

                Bgs[i] = Vector3d(para_SpeedBias[i][6],
                                  para_SpeedBias[i][7],
                                  para_SpeedBias[i][8]);
            
        }

        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5]).normalized().toRotationMatrix();
        }
        
        td = para_Td[0][0];
        //line 
        if(USE_STRUCT_LINE)
        {
            //如果有优化水平线，需要矫正local_mht
            double local_mht_last = local_mht;
            local_mht = para_Local_MHT[0][0];
            if(have_h_lines_opt)
            {
                local_mht += y_diff_rad;
                local_mht = normalizeAngle(local_mht);
                have_h_lines_opt = false;
                ROS_DEBUG("Local mht changing: %lf ===> %lf ===> %lf", local_mht_last, para_Local_MHT[0][0], local_mht);
            }

            MatrixXd sline_mat(struct_line_manager.getFeatureCount(), 2);
            for(int i = 0; i < sline_mat.rows(); i++)
            {
                double inv_depth = para_Struct_Line[i][0];
                double phi = para_Struct_Line[i][1];
                //垂直线段参数需要进行矫正
                if(struct_lines_opt_type[i] == VERTICAL)
                    phi = normalizeAngle(phi + y_diff_rad);

                sline_mat.row(i)[0] = inv_depth;
                sline_mat.row(i)[1] = phi;
            }
            struct_line_manager.setLineFeature(sline_mat);
        }
        else if(enable_line_opti || enable_triang_opti_only)
        {
            MatrixXd line_orth_mat(line_manager.getFeatureCount(), 4);
            for(int i = 0; i < line_orth_mat.rows(); i++)
            {
                //需要把优化后的线特征转到原来的世界坐标系（优化过程过程中由于yaw和position不可观导致世界坐标系向这些方向漂移）
                Vector4d line_wo_orth(para_Line[i][0], para_Line[i][1], para_Line[i][2], para_Line[i][3]);
                Vector6d line_wo_pluk = orthToPluk(line_wo_orth);
                Vector6d line_wn_pluk = plukTransformPose(line_wo_pluk, Rwn_wo, twn_wo);
                Vector4d orth = plukToOrth(line_wn_pluk);
                if(enable_triang_opti_only && !enable_line_opti)
                    line_orth_mat.row(i) = line_wo_orth;
                else 
                    line_orth_mat.row(i) = orth;
            }
            line_manager.setLineFeature(line_orth_mat);
        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);
}

bool Estimator::failureDetection()
{
    return false;
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        //return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        //ROS_INFO(" big translation");
        //return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        //ROS_INFO(" big z translation");
        //return true; 
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        //return true;
    }
    return false;
}

void Estimator::optimization()
{
    TicToc t_whole, t_prepare;
    vector2double();

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    //loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if(USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if(!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            //ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            //ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    problem.AddParameterBlock(para_Td[0], 1);

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);

    //marginalization factor
    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    //imu preintegration factor
    if(USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }
    //points reprojection factor
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }

            if(STEREO && it_per_frame.is_stereo)
            {                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else
                {
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                
            }
            f_m_cnt++;
        }
    }
    ROS_DEBUG("point measurements that add to ceres count: %d", f_m_cnt);

    //line reprojection factor
    if(USE_STRUCT_LINE)
    {
        ceres::LocalParameterization *local_parameterization = new MHTParameterization();
        problem.AddParameterBlock(para_Local_MHT[0], SIZE_MHT, local_parameterization);
        if(!ENABLE_MHT_OPT)
            problem.SetParameterBlockConstant(para_Local_MHT[0]);

        int line_m_cnt = 0;
        int line_h_cnt = 0;
        int line_v_cnt = 0;
        int line_index = -1;
        for(auto &it_per_id : struct_line_manager.struct_line_features)
        {
            it_per_id.used_num = it_per_id.line_feature_per_frame.size();
            if(!struct_line_manager.isLineUsable(it_per_id))
                continue;

            line_index++;

            if(it_per_id.line_type == VERTICAL)
                line_v_cnt++;
            else
            {
                line_h_cnt++;
                have_h_lines_opt = true; 
            }
            //add ParameterBlock
            ceres::LocalParameterization *local_parameterization = new StructLineParameterization();
            problem.AddParameterBlock(para_Struct_Line[line_index], SIZE_STRUCT_LINE, local_parameterization);

            //add Residual
            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            for(auto &it_per_frame : it_per_id.line_feature_per_frame)
            {
                imu_j++;
                if(imu_j == imu_i)
                {
                    // ceres::CostFunction *struct_line_factor = StructLineProjectionOneFrameFactor::create(it_per_frame.pt_start, it_per_frame.pt_end, 
                    //                                                                             it_per_frame.velocity_start, it_per_frame.velocity_end, 
                    //                                                                             it_per_frame.cur_td, it_per_id.line_type);
                    // problem.AddResidualBlock(struct_line_factor, loss_function,
                    //     para_Struct_Line[line_index],
                    //     para_Local_MHT[0],
                    //     para_Pose[imu_j],
                    //     para_Ex_Pose[0],
                    //     para_Td[0]);
                    if(it_per_id.line_type == VERTICAL)
                    {
                        ceres::CostFunction *vertical_line_factor = VerticalLineProjectionOneFrameFactor::create(it_per_frame.pt_start, it_per_frame.pt_end);
                        problem.AddResidualBlock(vertical_line_factor, loss_function,
                                                 para_Struct_Line[line_index],
                                                 para_Pose[imu_j],
                                                 para_Ex_Pose[0]);
                    }
                    else
                    {
                        ceres::CostFunction *horizon_line_factor = HorizonLineProjectionOneFrameFactor::create(it_per_frame.pt_start, it_per_frame.pt_end, it_per_id.line_type);
                        problem.AddResidualBlock(horizon_line_factor, loss_function,
                                                 para_Struct_Line[line_index],
                                                 para_Local_MHT[0],
                                                 para_Pose[imu_j],
                                                 para_Ex_Pose[0]);
                    }
                }
                else
                {
                    // ceres::CostFunction *struct_line_factor = StructLineProjectionTwoFrameFactor::create(it_per_frame.pt_start, it_per_frame.pt_end, 
                    //                                                                             it_per_frame.velocity_start, it_per_frame.velocity_end, 
                    //                                                                             it_per_frame.cur_td, it_per_id.line_type);
                    // problem.AddResidualBlock(struct_line_factor, loss_function,
                    //                         para_Struct_Line[line_index],
                    //                         para_Local_MHT[0],
                    //                         para_Pose[imu_i],
                    //                         para_Pose[imu_j],
                    //                         para_Ex_Pose[0],
                    //                         para_Td[0]);
                    if(it_per_id.line_type == VERTICAL)
                    {
                        ceres::CostFunction *vertical_line_factor = VerticalLineProjectionTwoFrameFactor::create(it_per_frame.pt_start, it_per_frame.pt_end);
                        problem.AddResidualBlock(vertical_line_factor, loss_function,
                                                 para_Struct_Line[line_index],
                                                 para_Pose[imu_i],
                                                 para_Pose[imu_j],
                                                 para_Ex_Pose[0]);
                    }
                    else
                    {
                        ceres::CostFunction *horizon_line_factor = HorizonLineProjectionTwoFrameFactor::create(it_per_frame.pt_start, it_per_frame.pt_end, it_per_id.line_type);
                        problem.AddResidualBlock(horizon_line_factor, loss_function,
                                                 para_Struct_Line[line_index],
                                                 para_Local_MHT[0],
                                                 para_Pose[imu_i],
                                                 para_Pose[imu_j],
                                                 para_Ex_Pose[0]);
                    }
                }
                line_m_cnt++;
            }
        }
        ROS_DEBUG("line measurements that add to ceres count: %d, line_v_cnt is %d, line_h_cnt is %d.", line_m_cnt, line_v_cnt, line_h_cnt);
    }
    else if(!USE_STRUCT_LINE && enable_line_opti)
    {
        int line_m_cnt = 0;
        int line_index = -1;
        for(auto &it_per_id : line_manager.line_features)
        {
            it_per_id.used_num = it_per_id.line_feature_per_frame.size();
            if (!(it_per_id.used_num >= line_min_obs && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulated))
                continue;
            
            line_index++;
            ceres::LocalParameterization *local_parameterization = new LineOrthParameterization();
            problem.AddParameterBlock(para_Line[line_index], SIZE_LINE, local_parameterization);

            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            for(auto &it_per_frame : it_per_id.line_feature_per_frame)
            {
                imu_j++;
                LineProjectionFactor *f = new LineProjectionFactor(it_per_frame.pt_start, it_per_frame.pt_end, 
                                                                it_per_frame.velocity_start, it_per_frame.velocity_end, it_per_frame.cur_td);
                problem.AddResidualBlock(f, loss_function, 
                                        para_Pose[imu_j], 
                                        para_Ex_Pose[0],
                                        para_Line[line_index],
                                        para_Td[0]);
                line_m_cnt++;
            } 
        }
        ROS_DEBUG("line measurements that add to ceres count: %d", line_m_cnt);
    }

    //printf("prepare for ceres: %f \n", t_prepare.toc());
    
    double slines_err_bf = calAllStructLinesReprojectionErrorAtZeroSpace();

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.BriefReport() << endl;//BriefReport or FullReport
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    printf("ceres solver costs: %f \n", t_solver.toc());

    double slines_err_af = calAllStructLinesReprojectionErrorAtZeroSpace();
    ROS_DEBUG("Struct lines rep err changing at ZERO Spece: %lf ===========> %lf", slines_err_bf, slines_err_af);

    double2vector();
    //printf("frame_count: %d \n", frame_count);

    if(frame_count < WINDOW_SIZE)
        return;

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if(USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if(imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if(STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if(imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }
        if(USE_STRUCT_LINE && ENABLE_STRUCT_LINE_MARGIN)
        {
            int line_index = -1;
            for(auto &it_per_id : struct_line_manager.struct_line_features)
            {
                it_per_id.used_num = it_per_id.line_feature_per_frame.size();
                if(!struct_line_manager.isLineUsable(it_per_id))
                    continue;

                line_index++;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if(imu_i != 0)
                    continue;
                for(auto &it_per_frame : it_per_id.line_feature_per_frame)
                {
                    imu_j++;
                    if(imu_j == imu_i)
                    {
                        // ceres::CostFunction *f = StructLineProjectionOneFrameFactor::create(it_per_frame.pt_start, it_per_frame.pt_end,
                        //                                                                     it_per_frame.velocity_start, it_per_frame.velocity_end,
                        //                                                                     it_per_frame.cur_td, it_per_id.line_type);
                        // ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                        //                                                                vector<double *>{para_Struct_Line[line_index], para_Local_MHT[0], para_Pose[imu_j], para_Ex_Pose[0], para_Td[0]},
                        //                                                                vector<int>{0, 2});
                        // marginalization_info->addResidualBlockInfo(residual_block_info);

                    }
                    else
                    {
                        // ceres::CostFunction *f = StructLineProjectionTwoFrameFactor::create(it_per_frame.pt_start, it_per_frame.pt_end,
                        //                                                                     it_per_frame.velocity_start, it_per_frame.velocity_end,
                        //                                                                     it_per_frame.cur_td, it_per_id.line_type);
                        // ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                        //                                                                vector<double *>{para_Struct_Line[line_index], para_Local_MHT[0], para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Td[0]},
                        //                                                                vector<int>{0, 2});
                        // marginalization_info->addResidualBlockInfo(residual_block_info);
                        if(it_per_id.line_type == VERTICAL)
                        {
                            ceres::CostFunction *f = VerticalLineProjectionTwoFrameFactor::create(it_per_frame.pt_start, it_per_frame.pt_end);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Struct_Line[line_index], para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0]},
                                                                                           vector<int>{0, 1});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ceres::CostFunction *f = HorizonLineProjectionTwoFrameFactor::create(it_per_frame.pt_start, it_per_frame.pt_end, it_per_id.line_type);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Struct_Line[line_index], para_Local_MHT[0], para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0]},
                                                                                           vector<int>{0, 2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }
        else if(!USE_STRUCT_LINE && enable_line_opti)
        {
            int line_index = -1;
            for(auto &it_per_id : line_manager.line_features)
            {
                it_per_id.used_num = it_per_id.line_feature_per_frame.size();
                if(!(it_per_id.used_num >= line_min_obs && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulated))
                    continue;

                line_index++;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if(imu_i != 0)
                    continue;
                for(auto &it_per_frame : it_per_id.line_feature_per_frame)
                {
                    imu_j++;
                    if(imu_i == imu_j)
                        continue;
                    LineProjectionFactor *f = new LineProjectionFactor(it_per_frame.pt_start, it_per_frame.pt_end, 
                                                               it_per_frame.velocity_start, it_per_frame.velocity_end, it_per_frame.cur_td);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function, 
                                                                                   vector<double *>{para_Pose[imu_j], para_Ex_Pose[0], para_Line[line_index], para_Td[0]},
                                                                                   vector<int>{2}); 
                    marginalization_info->addResidualBlockInfo(residual_block_info);                                                               
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if(USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        addr_shift[reinterpret_cast<long>(para_Local_MHT[0])] = para_Local_MHT[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else
    {//marge new
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            addr_shift[reinterpret_cast<long>(para_Local_MHT[0])] = para_Local_MHT[0];

            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    //printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    //printf("whole time for ceres: %f \n", t_whole.toc());
}

void Estimator::onlyLinesOptimization()
{
    vector2double();
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);
    //pose
    for(int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization); 
        problem.SetParameterBlockConstant(para_Pose[i]);
    }
    //ex pose
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
    //td
    problem.AddParameterBlock(para_Td[0], 1);
    problem.SetParameterBlockConstant(para_Td[0]);

    //add residual block
    int feature_index = -1;
    for(auto &it_per_id : line_manager.line_features)
    {
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if(!(it_per_id.used_num >= line_min_obs && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulated))
            continue;

        feature_index++;
        ceres::LocalParameterization *local_parameterization = new LineOrthParameterization();
        problem.AddParameterBlock(para_Line[feature_index], SIZE_LINE, local_parameterization);

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        for(auto &it_per_frame : it_per_id.line_feature_per_frame)
        {
            imu_j++;
            LineProjectionFactor *f = new LineProjectionFactor(it_per_frame.pt_start, it_per_frame.pt_end, 
                                                            it_per_frame.velocity_start, it_per_frame.velocity_end, it_per_frame.cur_td);
            problem.AddResidualBlock(f, loss_function,
                                    para_Pose[imu_j],
                                    para_Ex_Pose[0],
                                    para_Line[feature_index],
                                    para_Td[0]);
        }
    }

    if(feature_index < 3)
        return;
    TicToc tic_ol;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = NUM_ITERATIONS;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    ROS_DEBUG("onlyOptimize line cost %fms", tic_ol.toc());

    //double to vector
    MatrixXd line_orth_mat(line_manager.getFeatureCount(), 4);
    for(int i = 0; i < line_orth_mat.rows(); i++)
    {
        Vector4d orth(para_Line[i][0], para_Line[i][1], para_Line[i][2], para_Line[i][3]);
        line_orth_mat.row(i) = orth;
    }
    line_manager.setLineFeature(line_orth_mat);
}

void Estimator::onlyOptimizeMhtAndLines()
{
    vector2double();
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);
    //pose
    for(int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization); 
        problem.SetParameterBlockConstant(para_Pose[i]);
    }
    //ex pose
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
    //td
    problem.AddParameterBlock(para_Td[0], 1);
    problem.SetParameterBlockConstant(para_Td[0]);
    //local mht
    ceres::LocalParameterization *local_parameterization = new MHTParameterization();
    problem.AddParameterBlock(para_Local_MHT[0], SIZE_MHT, local_parameterization);
    if(!ENABLE_MHT_OPT)
        problem.SetParameterBlockConstant(para_Local_MHT[0]);

    //add residual block
    int feature_index = -1;
    for(auto &it_per_id : struct_line_manager.struct_line_features)
    {
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if(!struct_line_manager.isLineUsable(it_per_id))
            continue;
        feature_index++;
        //add ParameterBlock
        ceres::LocalParameterization *local_parameterization = new StructLineParameterization();
        problem.AddParameterBlock(para_Struct_Line[feature_index], SIZE_STRUCT_LINE, local_parameterization);
        //add Residual
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        for(auto &it_per_frame : it_per_id.line_feature_per_frame)
        {
            imu_j++;
            if(imu_j == imu_i)
            {
                ceres::CostFunction *struct_line_factor = StructLineProjectionOneFrameFactor::create(it_per_frame.pt_start, it_per_frame.pt_end, 
                                                                                            it_per_frame.velocity_start, it_per_frame.velocity_end, 
                                                                                            it_per_frame.cur_td, it_per_id.line_type);
                problem.AddResidualBlock(struct_line_factor, loss_function,
                                        para_Struct_Line[feature_index],
                                        para_Local_MHT[0],
                                        para_Pose[imu_j],
                                        para_Ex_Pose[0],
                                        para_Td[0]);
            }
            else
            {
                ceres::CostFunction *struct_line_factor = StructLineProjectionTwoFrameFactor::create(it_per_frame.pt_start, it_per_frame.pt_end, 
                                                                                            it_per_frame.velocity_start, it_per_frame.velocity_end, 
                                                                                            it_per_frame.cur_td, it_per_id.line_type);
                problem.AddResidualBlock(struct_line_factor, loss_function,
                                        para_Struct_Line[feature_index],
                                        para_Local_MHT[0],
                                        para_Pose[imu_i],
                                        para_Pose[imu_j],
                                        para_Ex_Pose[0],
                                        para_Td[0]);
            }
        }
    }

    if(feature_index < 3)
        return;
    TicToc tic_ol;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = NUM_ITERATIONS;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    ROS_DEBUG("onlyOptimizeMhtAndHline line cost %fms", tic_ol.toc());

    //double to vector
    local_mht = para_Local_MHT[0][0];
    MatrixXd sline_mat(struct_line_manager.getFeatureCount(), 2);
    for(int i = 0; i < sline_mat.rows(); i++)
    {
        sline_mat.row(i)[0] = para_Struct_Line[i][0];
        sline_mat.row(i)[1] = para_Struct_Line[i][1];
    }
    struct_line_manager.setLineFeature(sline_mat);
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);
                if(USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

            if(USE_IMU)
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];

            if(USE_IMU)
            {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
    if(USE_STRUCT_LINE)
    {
        struct_line_manager.removeFront(frame_count);
        if(mht_state == UPDATING)
            mht_manager.slideMHTWindowNew();
    }
    else
        line_manager.removeFront(frame_count);
}

void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);

        if(USE_STRUCT_LINE)
            struct_line_manager.removeBackShiftParam(P0, P1);
    }
    else
        f_manager.removeBack();

    if(!USE_STRUCT_LINE)
        line_manager.removeBack();
    else
    {
        if(mht_state == UPDATING)
            mht_manager.slideMHTWindowOld();
    }
}


void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame()
{
    //printf("predict pts in next frame\n");
    if(frame_count < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);
    map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : f_manager.feature)
    {
        if(it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            //printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            if((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
            {
                double depth = it_per_id.estimated_depth;
                Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                int ptsIndex = it_per_id.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }
    featureTracker.setPrediction(predictPts);
    //printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                 double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}
//两个端点到线的平均距离
double Estimator::lineReprojectionError(const Matrix3d &Ri, const Vector3d &Pi, const Vector3d &pt_start, const Vector3d &pt_end, const Vector6d &line_w_pluk)
{
    double err = 0;
    Matrix3d Rcw = Ri.transpose();
    Vector3d tcw = -Ri.transpose() * Pi;
    Vector6d line_c = plukTransformPose(line_w_pluk, Rcw, tcw);
    Vector3d nc = line_c.head(3);
    double line_norm = nc.head(2).norm();
    err += fabs(pt_start.dot(nc) / line_norm);
    err += fabs(pt_end.dot(nc) / line_norm);
    return err / 2.0;
}

void Estimator::outliersRejection(set<int> &removeIndex)
{
    //return;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index ++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;             
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                    Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                    depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            if(STEREO && it_per_frame.is_stereo)
            {
                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {            
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }       
            }
        }
        double ave_err = err / errCnt;
        if(ave_err * FOCAL_LENGTH > 3)//3个像素偏移
            removeIndex.insert(it_per_id.feature_id);

    }
}

void Estimator::lineOutliersRejection(set<int> &removeIndex)
{
    removeIndex.clear();
    for(auto &it_per_id : line_manager.line_features)
    {
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if(!(it_per_id.used_num >= line_min_obs && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulated))
            continue;
        int feature_id = it_per_id.feature_id;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector6d line_w_pluk = it_per_id.line_pluk;
        double max_err = 0;
        for(auto &it_per_frame : it_per_id.line_feature_per_frame)
        {
            imu_j++;
            Matrix3d Rwc = Rs[imu_j] * ric[0];
            Vector3d twc = Rs[imu_j] * tic[0] + Ps[imu_j];
            double err = lineReprojectionError(Rwc, twc, it_per_frame.pt_start, it_per_frame.pt_end, line_w_pluk);
            if(max_err < err)
                max_err = err;
        }
        if(max_err * FOCAL_LENGTH > outliers_thresh)
            removeIndex.insert(feature_id);
    }
}

void Estimator::structLineOutliersRejection(set<int> &removeIndex)
{
    removeIndex.clear();
    for(auto &it_per_id : struct_line_manager.struct_line_features)
    {
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if(!struct_line_manager.isLineUsable(it_per_id))
            continue;
        int feature_id = it_per_id.feature_id;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector6d line_w_pluk = it_per_id.getPlukInWorldFromParam(local_mht, Rs, Ps, tic, ric);
        double max_err = 0;
        for(auto &it_per_frame : it_per_id.line_feature_per_frame)
        {
            imu_j++;
            Matrix3d Rwc = Rs[imu_j] * ric[0];
            Vector3d twc = Rs[imu_j] * tic[0] + Ps[imu_j];
            double err = lineReprojectionError(Rwc, twc, it_per_frame.pt_start, it_per_frame.pt_end, line_w_pluk);
            if(max_err < err)
                max_err = err;
        }
        if(max_err * FOCAL_LENGTH > outliers_thresh)
            removeIndex.insert(feature_id);
    }
}

double Estimator::calAllStructLinesReprojectionError()
{
    double all_err = 0;
    for(auto &it_per_id : struct_line_manager.struct_line_features)
    {
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if(!struct_line_manager.isLineUsable(it_per_id))
            continue;
        int feature_id = it_per_id.feature_id;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector6d line_w_pluk = it_per_id.getPlukInWorldFromParam(local_mht, Rs, Ps, tic, ric);
        double max_err = 0;
        for(auto &it_per_frame : it_per_id.line_feature_per_frame)
        {
            imu_j++;
            Matrix3d Rwc = Rs[imu_j] * ric[0];
            Vector3d twc = Rs[imu_j] * tic[0] + Ps[imu_j];
            double err = lineReprojectionError(Rwc, twc, it_per_frame.pt_start, it_per_frame.pt_end, line_w_pluk);
            all_err += err;
        }
    }
    return all_err;   
}

double Estimator::calAllStructLinesReprojectionErrorAtZeroSpace()
{
    Matrix3d Rs_temp[WINDOW_SIZE+1];
    Vector3d Ps_temp[WINDOW_SIZE+1];
    Matrix3d ric_temp[2];
    Vector3d tic_temp[2];
    for(int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs_temp[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        Ps_temp[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
    }
    ric_temp[0] = Quaterniond(para_Ex_Pose[0][6], para_Ex_Pose[0][3], para_Ex_Pose[0][4], para_Ex_Pose[0][5]).normalized().toRotationMatrix();
    tic_temp[0] = Vector3d(para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2]);

    MatrixXd sline_mat(struct_line_manager.getFeatureCount(), 2);
    for(int i = 0; i < sline_mat.rows(); i++)
    {
        sline_mat.row(i)[0] = para_Struct_Line[i][0];
        sline_mat.row(i)[1] = para_Struct_Line[i][1];
    }
    struct_line_manager.setLineFeature(sline_mat);

    double local_mht_temp = para_Local_MHT[0][0];
    
    double all_err = 0;
    for(auto &it_per_id : struct_line_manager.struct_line_features)
    {
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if(!struct_line_manager.isLineUsable(it_per_id))
            continue;
        int feature_id = it_per_id.feature_id;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector6d line_w_pluk = it_per_id.getPlukInWorldFromParam(local_mht_temp, Rs_temp, Ps_temp, tic_temp, ric_temp);
        double max_err = 0;
        for(auto &it_per_frame : it_per_id.line_feature_per_frame)
        {
            imu_j++;
            Matrix3d Rwc = Rs_temp[imu_j] * ric_temp[0];
            Vector3d twc = Rs_temp[imu_j] * tic_temp[0] + Ps_temp[imu_j];
            double err = lineReprojectionError(Rwc, twc, it_per_frame.pt_start, it_per_frame.pt_end, line_w_pluk);
            all_err += err;
        }
    }
    return all_err;   
}

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

void Estimator::updateLatestStates()
{
    latest_time = Headers[frame_count] + td;
    latest_P = Ps[frame_count];
    latest_Q = Rs[frame_count];
    latest_V = Vs[frame_count];
    latest_Ba = Bas[frame_count];
    latest_Bg = Bgs[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    mBuf.lock();
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    while(!tmp_accBuf.empty())
    {
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }
    mBuf.unlock();
}
//计算imu预测位姿下的平均重投影误差
double Estimator::calCurRepErrAtImuPose(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &cur_features)
{
    double err = 0;
    int errCnt = 0;
    auto &feature = f_manager.feature;
    for (auto &id_pts : cur_features)
    {
        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId& id_f){
            return id_f.feature_id == feature_id;
        });
        if(it == feature.end()) continue;
        int used_num = it->feature_per_frame.size();
        if(used_num < 4) continue;

        Vector3d pts_j;
        pts_j.x() = id_pts.second[0].second(0);
        pts_j.y() = id_pts.second[0].second(1);
        pts_j.z() = id_pts.second[0].second(2);
        int imu_i = it->start_frame;
        double depth = it->estimated_depth;
        Vector3d pts_i = it->feature_per_frame[0].point; 
        err += reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                Rs[frame_count], Ps[frame_count], ric[0], tic[0],
                                depth, pts_i, pts_j);
        errCnt++;
    }
    if(!errCnt)
        return 0;
    err = err / errCnt * FOCAL_LENGTH;
    return err;
}
//计算最新帧的PNP位姿
void Estimator::calCurPoseByPNP(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &cur_features, Eigen::Matrix3d &R, Eigen::Vector3d &P)
{
    vector<cv::Point3f> pts3D;
    vector<cv::Point2f> pts2D;
    auto &feature = f_manager.feature;
    for(auto &id_pts : cur_features)
    {
        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId& id_f){
            return id_f.feature_id == feature_id;
        });
        if(it == feature.end()) continue;
        int used_num = it->feature_per_frame.size();
        if(used_num < 4) continue;

        Vector3d ptsInImu = ric[0] * (it->estimated_depth * it->feature_per_frame[0].point) + tic[0];
        Vector3d ptsInWorld = Rs[it->start_frame] * ptsInImu + Ps[it->start_frame];

        cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
        cv::Point2f point2d(id_pts.second[0].second(0), id_pts.second[0].second(1));
        pts3D.push_back(point3d);
        pts2D.push_back(point2d);
    }
    Matrix3d RCam = Rs[frame_count] * ric[0];
    Vector3d PCam = Rs[frame_count] * tic[0] + Ps[frame_count];
    if(f_manager.solvePoseByPnP(RCam, PCam, pts2D, pts3D))
    {
        R = RCam * ric[0].transpose();
        P = -RCam *ric[0].transpose() * tic[0] + PCam;
    }
}

void Estimator::calCurVelocity(double cur_time_, Vector3d &cur_P_)
{
    temp_cur_time = cur_time_;
    temp_cur_P = cur_P_;
    if(temp_last_time != 0)
    {
        double dt = temp_cur_time - temp_last_time;
        ROS_ASSERT(dt != 0);
        temp_cur_V = (temp_cur_P - temp_last_P) / dt;
        temp_cur_V_norm = temp_cur_V.norm();
    }
    temp_last_P = temp_cur_P;
    temp_last_time = temp_cur_time;
}
//比较两条线的相似度，如果足够相似则返回true+相似度分数
pair<bool, double> Estimator::getTwoLineSimScore(const Vector4d &line0, const Vector4d &line1)
{
    pair<bool, double> res(false, -1);
    double diff_ang = getTwoLinesAbsAngle(line0, line1);
    double diff_dist = getTwoLinesDistByP2L(line0, line1);
    diff_dist = normalToPixel(diff_dist);
    if(diff_ang < LINE_SIM_ANGLE_THRESH && diff_dist < LINE_SIM_DIST_THRESH)
    {
        res.first = true;
        res.second = (fabs(diff_ang - LINE_SIM_ANGLE_THRESH) / LINE_SIM_ANGLE_THRESH + fabs(diff_dist - LINE_SIM_DIST_THRESH) / LINE_SIM_DIST_THRESH) / 2;
    }
    return res;
}

//基于当前的局部曼哈顿划分线条,返回是否划分成功
bool Estimator::structLineClassify(const vector<pair<int, Eigen::Matrix<double, 8, 1>>> &new_lines, vector<pair<int, Eigen::Matrix<double, 8, 1>>> &out_lines, vector<LineType> &lines_type)
{
    out_lines.clear();
    lines_type.clear();
    int horizon_counts = 0;;
    int vertical_counts = 0;
    //calculate the vpz vpx vpy
    Matrix3d R_cw = ric[0].transpose() * Rs[frame_count].transpose();
    Vector3d DD_z(0, 0, 1);
    Vector3d DD_x(cos(local_mht), sin(local_mht), 0);
    Vector3d DD_y(-sin(local_mht), cos(local_mht), 0);
    DD_z = R_cw * DD_z;
    DD_x = R_cw * DD_x;
    DD_y = R_cw * DD_y;
    Vector2d vp_z = getVpFromDDs(DD_z);
    Vector2d vp_x = getVpFromDDs(DD_x);
    Vector2d vp_y = getVpFromDDs(DD_y);
    //classfy the line
    for(auto &it_per_id : new_lines)
    {
        vector<pair<int, double>> vec_score;
        Vector4d line_se = it_per_id.second.head(4);
        Vector2d mid_p = (it_per_id.second.head(2) + it_per_id.second.segment<2>(2)) / 2;
        Vector4d line_mvz, line_mvx, line_mvy;
        line_mvz << mid_p, vp_z;
        line_mvx << mid_p, vp_x;
        line_mvy << mid_p, vp_y;
        auto sim_z = getTwoLineSimScore(line_mvz, line_se);
        auto sim_x = getTwoLineSimScore(line_mvx, line_se);
        auto sim_y = getTwoLineSimScore(line_mvy, line_se);
        if(sim_z.first)
            vec_score.emplace_back(0, sim_z.second);
        if(sim_x.first)
            vec_score.emplace_back(1, sim_x.second);
        if(sim_y.first)
            vec_score.emplace_back(2, sim_y.second);
        //select the best match
        if(!vec_score.empty())
        {
            sort(vec_score.begin(), vec_score.end(), [](const pair<int, double> &a, const pair<int, double> &b){return a.second > b.second;});
            LineType cur_line_type = OTHER;
            switch(vec_score[0].first)
            {
            case 0:
                cur_line_type = VERTICAL;
                vertical_counts++;
                break;
            case 1:
                cur_line_type = HORIZON_X;
                horizon_counts++;
                break;
            case 2:
                cur_line_type = HORIZON_Y;
                horizon_counts++;
                break;
            default:
                break;
            }
            out_lines.push_back(it_per_id);
            lines_type.push_back(cur_line_type);
        }
    }
    int input_size = new_lines.size();
    int output_size = out_lines.size();
    ROS_DEBUG("structLineClassify: new lines input size %d, classified successfully counts %d, vertical counts is %d, horizon counts is %d.", input_size, output_size, vertical_counts, horizon_counts);
    double ratio = 1.0 * horizon_counts / (input_size - vertical_counts);
    if((input_size - vertical_counts) > 4 && ratio < NEW_MHT_DETECT_THRESH)
        return true;
    return false;
}
//只划分垂直线条，在局部曼哈顿更新时调用
void Estimator::onlyClassifyVerticalLine(const vector<pair<int, Eigen::Matrix<double, 8, 1>>> &new_lines, 
                                         vector<pair<int, Eigen::Matrix<double, 8, 1>>> &vertical_lines, vector<pair<int, Eigen::Matrix<double, 8, 1>>> &other_lines)
{
    vertical_lines.clear();
    other_lines.clear();
    Matrix3d R_cw = ric[0].transpose() * Rs[frame_count].transpose();
    Vector3d DD_z(0, 0, 1);
    DD_z = R_cw * DD_z;
    Vector2d vp_z = getVpFromDDs(DD_z);
    for(auto it_per_id : new_lines)
    {
        Vector4d line_se = it_per_id.second.head(4);
        Vector2d mid_p = (it_per_id.second.head(2) + it_per_id.second.segment<2>(2)) / 2;
        Vector4d line_mvz;
        line_mvz << mid_p, vp_z;
        auto sim_z = getTwoLineSimScore(line_mvz, line_se);
        if(sim_z.first)
            vertical_lines.push_back(it_per_id);
        else
            other_lines.push_back(it_per_id);
    }
    ROS_DEBUG("onlyClassifyVerticalLine: Input %d new lines, classified %d vertical lines, %d other lines", new_lines.size(), vertical_lines.size(), other_lines.size());
}
//给定局部曼哈顿对水平线条进行划分
void Estimator::onlyClassifyHorizonLine(double ransac_local_mht, const vector<pair<int, Eigen::Matrix<double, 8, 1>>> &h_lines, vector<pair<int, Eigen::Matrix<double, 8, 1>>> &out_lines, vector<LineType> &h_lines_type)
{
    out_lines.clear();
    h_lines_type.clear();
    Matrix3d R_cw = ric[0].transpose() * Rs[frame_count].transpose();
    Vector3d DD_x(cos(ransac_local_mht), sin(ransac_local_mht), 0);
    Vector3d DD_y(-sin(ransac_local_mht), cos(ransac_local_mht), 0);
    DD_x = R_cw * DD_x;
    DD_y = R_cw * DD_y;
    Vector2d vp_x = getVpFromDDs(DD_x);
    Vector2d vp_y = getVpFromDDs(DD_y);
    for(auto &it_per_id : h_lines)
    {
        vector<pair<int, double>> vec_score;
        Vector4d line_se = it_per_id.second.head(4);
        Vector2d mid_p = (it_per_id.second.head(2) + it_per_id.second.segment<2>(2)) / 2;
        Vector4d line_mvx, line_mvy;
        line_mvx << mid_p, vp_x;
        line_mvy << mid_p, vp_y;
        auto sim_x = getTwoLineSimScore(line_mvx, line_se);
        auto sim_y = getTwoLineSimScore(line_mvy, line_se);
        if(sim_x.first)
            vec_score.emplace_back(1, sim_x.second);
        if(sim_y.first)
            vec_score.emplace_back(2, sim_y.second);
        if(!vec_score.empty())
        {
            sort(vec_score.begin(), vec_score.end(), [](const pair<int, double> &a, const pair<int, double> &b){return a.second > b.second;});
            LineType cur_line_type = OTHER;
            switch(vec_score[0].first)
            {
            case 1:
                cur_line_type = HORIZON_X;
                break;
            case 2:
                cur_line_type = HORIZON_Y;
                break;
            default:
                break;
            }
            out_lines.push_back(it_per_id);
            h_lines_type.push_back(cur_line_type);
        }
    }
    ROS_DEBUG("onlyClassifyHorizonLine: Input %d lines, classified %d horizon lines at the new mht:%lf", h_lines.size(), out_lines.size(), ransac_local_mht);
}
//
int Estimator::countNumForHorizonClassify(const vector<pair<int, Eigen::Matrix<double, 8, 1>>> &cur_lines_all, const Vector2d &vp_x, const Vector2d &vp_y)
{
    int counts = 0;
    for(auto &it_per_id : cur_lines_all)
    {
        Vector4d line_se = it_per_id.second.head(4);
        Vector2d mid_p = (it_per_id.second.head(2) + it_per_id.second.segment<2>(2)) / 2;
        Vector4d line_mvx, line_mvy;
        line_mvx << mid_p, vp_x;
        line_mvy << mid_p, vp_y;
        auto sim_x = getTwoLineSimScore(line_mvx, line_se);
        auto sim_y = getTwoLineSimScore(line_mvy, line_se);
        if(sim_x.first || sim_y.first)
            counts++;
    }
    return counts;
}
//RANSAC计算新MHT，输入cur_lines_all需要已剔除垂直线
pair<bool, double> Estimator::recognizeMHTUsingRANSAC(int frame_count, const vector<pair<int, Eigen::Matrix<double, 8, 1>>> &cur_lines_all)
{
    if(cur_lines_all.empty())
        return pair<bool, double>(false, 0);
    ROS_DEBUG("recognizeMHTUsingRANSAC: Input %d lines", cur_lines_all.size());
    pair<bool, double> res;
    //cal the vanishing line of xy plane
    Matrix3d R_cw = ric[0].transpose() * Rs[frame_count].transpose();
    Vector3d l_v = R_cw * Vector3d(0, 0, 1.0);
    Vector3d z_w = Vector3d(0, 0, 1.0);
    ROS_DEBUG("recognizeMHTUsingRANSAC: l_v is (%lf, %lf, %lf)", l_v(0), l_v(1), l_v(2));
    //gen rand
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    //ransac
    double optimal_mht = 0;
    Vector3d optimal_vpx;
    int max_classfied_counts = 0;
    int iteration_counts = RANSAC_MAX_ITERATIONS;
    while(iteration_counts--)
    {
        double random_number = dis(gen);
        int select_index = static_cast<int>(std::floor(random_number * cur_lines_all.size()));
        Vector4d cur_line = cur_lines_all[select_index].second.head(4);
        ROS_DEBUG("recognizeMHTUsingRANSAC: select_index is %d, cur_line is (%lf, %lf, %lf, %lf)", select_index, cur_line(0), cur_line(1), cur_line(2), cur_line(3));
        //cal vpx_w and vpy_w
        Vector3d l_cur = getLineExpression(cur_line);
        ROS_DEBUG("recognizeMHTUsingRANSAC: l_cur is (%lf, %lf, %lf)", l_cur(0), l_cur(1), l_cur(2));
        Vector2d vpx_l = getIntersecByTwoLine(l_v, l_cur);

        Vector3d vpx_l_temp;
        vpx_l_temp << vpx_l, 1.0;
        double vp_l_its_e_1 = l_v.dot(vpx_l_temp);
        double vp_l_its_e_2 = l_cur.dot(vpx_l_temp);
        ROS_DEBUG("recognizeMHTUsingRANSAC: vp_l_its_e_1 is %lf, vp_l_its_e_2 is %lf", vp_l_its_e_1, vp_l_its_e_2);

        if(std::isinf(vpx_l(0)) || std::isinf(vpx_l(1)) || std::isnan(vpx_l(0)) || std::isnan(vpx_l(1)))
            continue;
        Vector3d vpx_n;
        vpx_n << vpx_l, 1.0;
        vpx_n.normalize();
        Vector3d vpx_w = R_cw.transpose() * vpx_n;
        ROS_DEBUG("recognizeMHTUsingRANSAC: current vanish point is (%lf, %lf), vpx_n is (%lf, %lf, %lf), vpx_w is (%lf, %lf, %lf)", vpx_l(0), vpx_l(1), vpx_n(0), vpx_n(1), vpx_n(2), vpx_w(0), vpx_w(1), vpx_w(2));
        vpx_w(2) = 0;
        vpx_w = vpxNormalize(vpx_w);
        Vector3d vpy_w = z_w.cross(vpx_w);
        //cal new vpx and vpy in image normalized coordinate
        Vector3d vpx_c = R_cw * vpx_w;
        Vector3d vpy_c = R_cw * vpy_w;
        Vector2d vpx_r = getVpFromDDs(vpx_c);
        Vector2d vpy_r = getVpFromDDs(vpy_c);
        ROS_DEBUG("recognizeMHTUsingRANSAC: vanish point after normalized is :vpx_r(%lf, %lf), vpy_r(%lf, %lf)", vpx_r(0), vpx_r(1), vpy_r(0), vpy_r(1));
        //cal counts for curretn vp
        int cur_counts = countNumForHorizonClassify(cur_lines_all, vpx_r, vpy_r);
        ROS_DEBUG("recognizeMHTUsingRANSAC: current counts based on new normalized vp is %d", cur_counts);
        double cur_mht = atan2(vpx_w(1), vpx_w[0]);
        if(cur_counts > max_classfied_counts)
        {
            optimal_mht = cur_mht;
            optimal_vpx = vpx_w;
            max_classfied_counts = cur_counts;
        }
    }
    res.first = true;
    res.second = optimal_mht;
    ROS_DEBUG("MHT-RANSAC: the optimal_mht is %lf[deg], optimal_vpx is (%f, %f, %f).", optimal_mht / M_PI * 180.0, optimal_vpx(0), optimal_vpx(1), optimal_vpx(2));
    return res;
}
//将vpx_w限制在第一象限
Vector3d Estimator::vpxNormalize(Vector3d vpx_in)
{
    Vector3d vpx_out;
    vpx_in(2) = 0; //强制投影在xy平面上
    double phi = atan2(vpx_in(1), vpx_in(0));
    ROS_DEBUG("vpxNormalize: original phi is %lf", phi);
    Vector3d v_z = Vector3d(0, 0, 1.0);
    if(phi >= 0 && phi <= (M_PI/2))
    {
        vpx_out = vpx_in;
    }
    else if(phi > (M_PI/2))
    {
        vpx_out = vpx_in.cross(v_z);
    }
    else if(phi < 0 && phi >= (-M_PI/2))
    {
        vpx_out = v_z.cross(vpx_in);
    }
    else
    {
        vpx_out = -vpx_in;
    }
    //如果vpx_w趋近与90°，则将其归0
    double normalized_phi = atan2(vpx_out(1), vpx_out(0));
    assert(normalized_phi >= 0 && normalized_phi <=(M_PI/2));
    if((M_PI/2 - normalized_phi) < 0.05236)//0.0872：5° 0.05236:3°
    {
        normalized_phi = 0.;
        vpx_out = Vector3d(1.0, 0, 0);
    }
    
    ROS_DEBUG("vpxNormalize: normalized_phi is %lf", normalized_phi);
    return vpx_out;
}

Vector2d Estimator::getVpxFromCurLMHT()
{
    Matrix3d R_wc = Rs[frame_count] * ric[0];
    Vector3d vpx_w = Vector3d(cos(local_mht), sin(local_mht), 0);
    Vector3d vpx_c = R_wc.transpose() * vpx_w;
    Vector2d vpx = getVpFromDDs(vpx_c);//vanish point
    return vpx;
}
Vector2d Estimator::getVpyFromCurLMHT()
{
    Matrix3d R_wc = Rs[frame_count] * ric[0];
    Vector3d vpy_w = Vector3d(-sin(local_mht), cos(local_mht), 0);
    Vector3d vpy_c = R_wc.transpose() * vpy_w;
    Vector2d vpy = getVpFromDDs(vpy_c);//vanish point
    return vpy;
}
//得到自适应主消失点，用于可视化
Vector2d Estimator::getAdaptiveVp()
{
    Matrix3d R_wc = Rs[frame_count] * ric[0];
    Vector3d vpx_w = Vector3d(cos(local_mht), sin(local_mht), 0);
    Vector3d vpx_c = R_wc.transpose() * vpx_w;
    if(fabs(vpx_c.z()) < 0.25)
    {
        Vector3d vpy_w = Vector3d(-sin(local_mht), cos(local_mht), 0);
        Vector3d vpy_c = R_wc.transpose() * vpy_w;
        vpx_c = vpy_c;
    }   
    Vector2d vpx = getVpFromDDs(vpx_c);//vanish point
    return vpx;
}

void Estimator::calAssociaPtsForLines(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &cur_pts, const vector<pair<int, Vector4d>> &lines, vector<pair<int, vector<pair<int, double>>>> &associa_pts)
{
    int idx = 0; 
    MatrixXd cur_pts_mat(cur_pts.size(), 3);
    vector<int> pts_id;
    for(auto &it_per_id : cur_pts)
    {
        Vector3d pt = it_per_id.second[0].second.head(3);
        pt /= pt(2);
        cur_pts_mat.row(idx++) = pt;
        pts_id.push_back(it_per_id.first);
    }
    associa_pts.clear();
    for(auto &line : lines)
    {
        double lx1 = line.second(0);
        double ly1 = line.second(1);
        double lx2 = line.second(2);
        double ly2 = line.second(3);
        double min_lx = lx1;
        double max_lx = lx2;
        double min_ly = ly1;
        double max_ly = ly2;
        if(lx1 > lx2) std::swap(min_lx, max_lx);
        if(ly1 > ly2) std::swap(min_ly, max_ly);
        //cal distance
        Vector3d l_param = getLineExpression(line.second);
        double l_norm = l_param.head(2).norm();
        VectorXd pts_l_dist = cur_pts_mat * l_param / l_norm;
        pts_l_dist = pts_l_dist.cwiseAbs();
        //associate pts
        vector<pair<int, double>> pt_id_dist;
        for(int i = 0; i < pts_l_dist.rows(); i++)
        {
            if(pts_l_dist(i) > pixelToNormal(3))
                continue;

            double px = cur_pts_mat(i, 0);
            double py = cur_pts_mat(i, 1);
            if(px < min_lx - pixelToNormal(3) || px > max_lx + pixelToNormal(3) || py < min_ly - pixelToNormal(3) || py > max_ly + pixelToNormal(3)) 
                continue;

            double side1 = std::pow((lx1 - px), 2) + std::pow((ly1 - py), 2);
            double side2 = std::pow((lx2 - px), 2) + std::pow((ly2 - py), 2);
            double line_side = std::pow(l_norm, 2);
            if(side1 <= pixelToNormal(9) || side2 <= pixelToNormal(9) || ((side1 < line_side + side2) && (side2 < line_side + side1)))
                pt_id_dist.emplace_back(pts_id[i], pts_l_dist[i]);
        }
        associa_pts.emplace_back(line.first, pt_id_dist);
        //ROS_DEBUG("calAssociaPtsForLines: line-%d associate %d points", line.first, pt_id_dist.size());
    }
}

cv::Mat Estimator::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = ptr->image.clone();
    return img;
}

void Estimator::DrawImage(double cur_header)
{
    cv::Mat cur_img;
    {
        std::lock_guard<std::mutex> lck(mtx_img_buf);
        while(!img0_buf.empty() && img0_buf.front()->header.stamp.toSec() < cur_header)
        {
            img0_buf.pop();
        }
        if(img0_buf.empty())
            return;
        if(img0_buf.front()->header.stamp.toSec() != cur_header)
        {
            img0_buf.pop();
            return;
        }
        cur_img = getImageFromMsg(img0_buf.front());
        img0_buf.pop();
    }
    //draw a string
    //text1
    string text1;
    if(mht_state == HOLD)
    {
        text1.append("LMW orientation: ");
        double local_mht_d = local_mht / M_PI * 180.;
        text1.append(std::to_string(local_mht_d));
        text1.append(" deg");
    }
    else
    {
        text1.append("LMW Updating...");
        // double local_mht_d = mht_manager.getLatestMHT() / M_PI * 180.;
        // text1.append(std::to_string(local_mht_d));
    }
    int fontFace = cv::FONT_HERSHEY_COMPLEX_SMALL; // 字体类型 FONT_HERSHEY_COMPLEX_SMALL  FONT_HERSHEY_SIMPLEX
    double fontScale = 1.0;                  // 字体缩放比例
    int thickness = 1;                       // 线条粗细
    cv::Scalar color(0, 255, 0);             // 绿色（BGR格式）
    int baseline;
    cv::Size text1Size = cv::getTextSize(text1, fontFace, fontScale, thickness, &baseline);

    int margin = 10;
    int x1 = cur_img.cols - text1Size.width - margin;
    int y1 = cur_img.rows - margin;
    cv::Point textOrg1(x1, y1);
    cv::putText(cur_img, text1, textOrg1, fontFace, fontScale, color, thickness);

    //text2
    string text2;
    text2.append("yaw orientation: ");
    Vector3d cur_ypr = Utility::R2ypr(Rs[frame_count]);
    double yaw_deg = cur_ypr[0];
    text2.append(std::to_string(yaw_deg));
    text2.append(" deg");

    cv::Size text2Size = cv::getTextSize(text2, fontFace, fontScale, thickness, &baseline);
    int x2 = cur_img.cols - text2Size.width - margin;
    int y2 = y1 - text1Size.height - 5;
    cv::Point textOrg2(x2, y2);
    cv::putText(cur_img, text2, textOrg2, fontFace, fontScale, color, thickness);

    //draw arrow
    if(mht_state == HOLD)
    {
        Vector2d vpx = getAdaptiveVp();
        double fx = 389.6706237792969;
        double fy = 389.6706237792969;
        double cx =  323.40972900390625;
        double cy = 232.05543518066406;

        Vector2d st_uv;
        st_uv << (640. / 2.), (480. - 100.);
        Vector2d st_xy  = Vector2d((st_uv[0] - cx) / fx, (st_uv[1] - cy) / fy);
        Vector2d direction = (vpx - st_xy).normalized();
        Vector2d end_uv = st_uv + 60 * direction;

        cv::Point2f st(st_uv[0], st_uv[1]);
        cv::Point2f ed(end_uv[0], end_uv[1]);
        cv::arrowedLine(cur_img, st, ed, cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0, 0.1);
    }

    imTrack = cur_img.clone();
}