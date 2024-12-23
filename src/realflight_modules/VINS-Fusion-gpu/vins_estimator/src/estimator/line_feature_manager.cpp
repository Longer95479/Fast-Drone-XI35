#include "line_feature_manager.h"

void LineFeatureManager::addLineFeature(int frame_cnt, const map<int, Eigen::Matrix<double, 8, 1>> &img_line, double td)
{
    for(auto &it_line : img_line)
    {
        int line_id = it_line.first;
        auto it = find_if(line_features.begin(), line_features.end(), [line_id](const LineFeaturePerId &line){
            return line.feature_id == line_id;
        });
        if(it != line_features.end())
        {
            it->pushFrame(it_line.second, td);
        }
        else
        {
            line_features.push_back(LineFeaturePerId(line_id, frame_cnt));
            line_features.back().pushFrame(it_line.second, td);
        }
    }
}

void LineFeatureManager::line_triangulate(Matrix3d Rs[], Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    int counts = 0;
    for(auto &it_per_id : line_features)
    {
        if(it_per_id.is_triangulated)
            continue;
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if(!(it_per_id.used_num >= line_min_obs && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        double min_cosine_dist = 1;
        Vector4d pi_i, pi_j;
        Vector3d n_pi_i;
        int imu_i = it_per_id.start_frame;
        int imu_j = imu_i - 1;
        for(auto &it_per_frame : it_per_id.line_feature_per_frame)
        {
            imu_j++;
            Matrix3d R_wc = Rs[imu_j] * ric[0];
            Vector3d t_wc = Rs[imu_j] * tic[0] + Ps[imu_j];
            Vector3d p1_w = R_wc * it_per_frame.pt_start + t_wc;
            Vector3d p2_w = R_wc * it_per_frame.pt_end + t_wc;
            Vector3d p3_w = t_wc;
            // ROS_DEBUG("line-%d at %d frame's start is (%lf, %lf, %lf), end is (%lf, %lf, %lf)", it_per_id.feature_id, imu_j, 
            //         it_per_frame.pt_start[0], it_per_frame.pt_start[1], it_per_frame.pt_start[2], 
            //         it_per_frame.pt_end[0], it_per_frame.pt_end[1], it_per_frame.pt_end[2]);
            // ROS_DEBUG("p1_w:(%lf, %lf, %lf), p2_w:(%lf, %lf, %lf), p3_w:(%lf, %lf, %lf)", p1_w[0], p1_w[1], p1_w[2], p2_w[0], p2_w[1], p2_w[2], p3_w[0], p3_w[1], p3_w[2]);
            if(imu_j == imu_i)
            {
                pi_i = pointsToPlane(p3_w, p1_w, p2_w);
                n_pi_i = pi_i.head(3).normalized();
                ROS_DEBUG("line-%d has %d frame", it_per_id.feature_id, it_per_id.used_num);
                ROS_DEBUG("pi_i:(%lf, %lf, %lf ,%lf)", pi_i[0], pi_i[1], pi_i[2], pi_i[3]);
                continue;
            }
            //select the plane that has the minimum cosine distance with the pi_i
            Vector4d pi_j_candi = pointsToPlane(p3_w, p1_w, p2_w);
            Vector3d n_pi_j = pi_j_candi.head(3).normalized();
            double n_cosine_dist = fabs(n_pi_i.dot(n_pi_j));
            if(n_cosine_dist < min_cosine_dist)
            {
                min_cosine_dist = n_cosine_dist;
                pi_j = pi_j_candi;
                ROS_DEBUG("pi_j:(%lf, %lf, %lf, %lf), cosine_dist is %lf", pi_j[0], pi_j[1], pi_j[2], pi_j[3], n_cosine_dist);
            }
        }
        if(min_cosine_dist > line_max_cosine_dist)
            continue;

        it_per_id.line_pluk = planesToLine(pi_i, pi_j);
        it_per_id.is_triangulated = true;
        counts++;
        ROS_DEBUG("line id-%d triangulate result is (%lf, %lf, %lf, %lf, %lf, %lf)", it_per_id.feature_id, it_per_id.line_pluk[0], it_per_id.line_pluk[1], it_per_id.line_pluk[2], 
                                                                                                          it_per_id.line_pluk[3], it_per_id.line_pluk[4], it_per_id.line_pluk[5]);
    }
    ROS_DEBUG("line triangulate successfully counts: %d", counts);
}

void LineFeatureManager::removeBack()
{
    for(auto it = line_features.begin(), it_next = line_features.begin(); it != line_features.end(); it = it_next)
    {
        it_next++;
        if(it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->line_feature_per_frame.erase(it->line_feature_per_frame.begin());
            it->used_num = it->line_feature_per_frame.size();
            if(it->used_num == 0)
                line_features.erase(it);
        }
    }
}

void LineFeatureManager::removeFront(int frame_count)
{
    for(auto it = line_features.begin(), it_next = line_features.begin(); it != line_features.end(); it = it_next)
    {
        it_next++;
        if(it->start_frame == frame_count)
            it->start_frame--;
        else
        {
            if(it->endFrame() < frame_count - 1)
                continue;
            int j = frame_count - 1 - it->start_frame;
            it->line_feature_per_frame.erase(it->line_feature_per_frame.begin() + j);
            it->used_num = it->line_feature_per_frame.size();
            if(it->used_num == 0)
                line_features.erase(it);
        }
    }
}

int LineFeatureManager::getFeatureCount()
{
    int cnt = 0;
    for(auto &it_per_id : line_features)
    {
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if(it_per_id.used_num >= line_min_obs && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulated)
            cnt++;
    }
    return cnt;
}

MatrixXd LineFeatureManager::getLineOrthMat()
{
    MatrixXd line_orth_mat(getFeatureCount(), 4);
    int index = 0;
    for(auto &it_per_id : line_features)
    {
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if(!(it_per_id.used_num >= line_min_obs && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulated))
            continue;
        line_orth_mat.row(index++) = plukToOrth(it_per_id.line_pluk);
    }
    return line_orth_mat;
}

void LineFeatureManager::setLineFeature(const MatrixXd &lineOrthMat)
{
    int index = 0;
    for(auto &it_per_id : line_features)
    {
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if(!(it_per_id.used_num >= line_min_obs && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulated))
            continue;
        it_per_id.line_pluk = orthToPluk(lineOrthMat.row(index++));
    }
}

void LineFeatureManager::removeOutlier(set<int> &outlierIndex)
{
    std::set<int>::iterator itSet;
    for (auto it = line_features.begin(), it_next = line_features.begin();
         it != line_features.end(); it = it_next)
    {
        it_next++;
        int index = it->feature_id;
        itSet = outlierIndex.find(index);
        if(itSet != outlierIndex.end())
        {
            line_features.erase(it);
            //printf("remove line outlier %d \n", index);
        }
    }
}