#include "line_feature_manager.h"
#include <algorithm>

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

/*********************************************************struct line*********************************************************/
bool StructLineFeatureManager::isLineUsable(const StructLineFeaturePerId& line)
{
    if(line.line_feature_per_frame.size() >= line_min_obs && line.start_frame < WINDOW_SIZE - 2 && line.is_triangulated)
        return true;
    return false;
}

//添加已存在的结构线条，返回新的线条
void StructLineFeatureManager::addTrackedStructLine(const map<int, Eigen::Matrix<double, 8, 1>> &img_line, double td, vector<pair<int, Eigen::Matrix<double, 8, 1>>> &new_lines)
{
    new_lines.clear();
    int tracked_counts = 0;
    for(auto& line_per_id : img_line)
    {
        int line_id = line_per_id.first;
        auto it = find_if(struct_line_features.begin(), struct_line_features.end(), 
                    [line_id](const StructLineFeaturePerId& line){return line.feature_id == line_id;});
        if(it != struct_line_features.end())
        {
            tracked_counts++;
            it->pushFrame(line_per_id.second, td);
        }
        else
        {
            new_lines.emplace_back(line_id, line_per_id.second);
        }
    }
    ROS_DEBUG("addTrackedStructLine: Input %d lines, %d tracked, %d new lines.", img_line.size(), tracked_counts, new_lines.size());
}
//添加已存在的结构线条，返回追踪到的水平线以及新线段
void StructLineFeatureManager::addTrackedStructLineAndGetHorizon(const map<int, Eigen::Matrix<double, 8, 1>> &img_line, double td,
     vector<pair<int, Eigen::Matrix<double, 8, 1>>> &h_lines, vector<pair<int, Eigen::Matrix<double, 8, 1>>> &new_lines)
{
    h_lines.clear();
    new_lines.clear();
    int tracked_counts = 0;
    for(auto& it_per_id : img_line)
    {
        int line_id = it_per_id.first;
        auto it = find_if(struct_line_features.begin(), struct_line_features.end(), 
            [line_id](const StructLineFeaturePerId& line){return line.feature_id == line_id;});
        if(it != struct_line_features.end())
        {
            tracked_counts++;
            it->pushFrame(it_per_id.second, td);
            if(it->line_type == HORIZON_X || it->line_type == HORIZON_Y)
            {
                h_lines.emplace_back(line_id, it_per_id.second);
            }
        }
        else
        {
            new_lines.emplace_back(line_id, it_per_id.second);
        }
    }
    ROS_DEBUG("addTrackedStructLineAndGetHorizon: Input %d lines, %d tracked, %d tracked horizon, %d new lines.", img_line.size(), tracked_counts, h_lines.size(), new_lines.size());
}
//向list中添加新的线条
void StructLineFeatureManager::addNewStrcutLine(int frame_cnt, const vector<pair<int, Eigen::Matrix<double, 8, 1>>> &new_lines, 
                                                const vector<LineType> &lines_type, double td)
{
    for(int i = 0; i < new_lines.size(); i++)
    {
        int new_line_id = new_lines[i].first;
        auto it = find_if(struct_line_features.begin(), struct_line_features.end(), [new_line_id](const StructLineFeaturePerId &line){return line.feature_id == new_line_id;});
        if(it != struct_line_features.end())
            continue;
        struct_line_features.push_back(StructLineFeaturePerId(new_lines[i].first, frame_cnt, lines_type[i]));
        struct_line_features.back().pushFrame(new_lines[i].second, td);
    }
}

void StructLineFeatureManager::structLineTriangulate(double local_mht, Matrix3d Rs[], Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    int counts = 0;
    for(auto &it_per_id : struct_line_features)
    {
        if(it_per_id.is_triangulated)
            continue;
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if(!(it_per_id.used_num >= line_min_obs && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if(it_per_id.line_type == OTHER)
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
            if(imu_j == imu_i)
            {
                pi_i = pointsToPlane(p3_w, p1_w, p2_w);
                n_pi_i = pi_i.head(3).normalized();
                // ROS_DEBUG("line-%d has %d frame", it_per_id.feature_id, it_per_id.used_num);
                // ROS_DEBUG("pi_i:(%lf, %lf, %lf ,%lf)", pi_i[0], pi_i[1], pi_i[2], pi_i[3]);
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
                // ROS_DEBUG("pi_j:(%lf, %lf, %lf, %lf), cosine_dist is %lf", pi_j[0], pi_j[1], pi_j[2], pi_j[3], n_cosine_dist);
            }
        }
        if(min_cosine_dist > line_max_cosine_dist)
            continue;
        it_per_id.line_pluk = planesToLine(pi_i, pi_j);
        //initialize param
        Vector3d t_wc = Rs[it_per_id.start_frame] * tic[0] + Ps[it_per_id.start_frame];
        Vector2d line_param = lineParamInitializationByPluk(local_mht, t_wc, it_per_id.line_pluk, it_per_id.line_type);
        it_per_id.setParam(line_param);
        it_per_id.is_triangulated = true;
        counts++;
        ROS_DEBUG("struct line id-%d triangulate result is (%lf, %lf, %lf, %lf, %lf, %lf)", it_per_id.feature_id, it_per_id.line_pluk[0], it_per_id.line_pluk[1], it_per_id.line_pluk[2], 
                                                                                                          it_per_id.line_pluk[3], it_per_id.line_pluk[4], it_per_id.line_pluk[5]);
    }
    ROS_DEBUG("struct line triangulate successfully counts: %d", counts);
}

void StructLineFeatureManager::structLineTriangulateByPoints(double local_mht, const FeatureManager &f_manager, Matrix3d Rs[], Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    int counts = 0;
    for(auto &it_per_id : struct_line_features)
    {
        if(it_per_id.is_triangulated || it_per_id.line_type == OTHER)
            continue;
        if(it_per_id.associa_points.size() < 2)
            continue;
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if(!(it_per_id.used_num >= line_min_obs && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        vector<pair<int, double>> id_pts;
        for(auto it_id_dist : it_per_id.associa_points)
        {
            id_pts.emplace_back(it_id_dist.first, it_id_dist.second);
        }
        sort(id_pts.begin(), id_pts.end(), [](const pair<int, double> &i1, const pair<int, double> &i2){return i1.second < i2.second;});
        //找到满足条件的两个点
        bool pt1_find = false, pt2_find = false;
        Vector3d pt1_w, pt2_w;
        for(auto it_id_dist : id_pts)
        {
            if(!pt1_find)
            {
                int pt1_id = it_id_dist.first;
                auto it = find_if(f_manager.feature.begin(), f_manager.feature.end(), [pt1_id](const FeaturePerId &p){return p.feature_id == pt1_id;});
                if(it == f_manager.feature.end() || it->solve_flag != 1)
                    continue;
                Vector3d pt_uv = it->feature_per_frame[0].point;
                int imu_i = it->start_frame;
                pt1_w = Rs[imu_i]* (ric[0] * (it->estimated_depth * pt_uv) + tic[0]) + Ps[imu_i];
                pt1_find = true;
                ROS_DEBUG("structLineTriangulateByPoints: find pt1 for line-%d: (%lf, %lf, %lf), line type is %d", it_per_id.feature_id, pt1_w(0), pt1_w(1), pt1_w(2), it_per_id.line_type);
                continue;
            }
            if(!pt2_find)
            {
                int pt2_id = it_id_dist.first;
                auto it = find_if(f_manager.feature.begin(), f_manager.feature.end(), [pt2_id](const FeaturePerId &p){return p.feature_id == pt2_id;});
                if(it == f_manager.feature.end() || it->solve_flag != 1)
                    continue;
                Vector3d pt_uv = it->feature_per_frame[0].point;
                int imu_i = it->start_frame;
                pt2_w = Rs[imu_i]* (ric[0] * (it->estimated_depth * pt_uv) + tic[0]) + Ps[imu_i];

                double p1_p2_dist = (pt1_w - pt2_w).norm();
                if(p1_p2_dist < 0.1)
                    continue;

                if(it_per_id.line_type == VERTICAL)
                {
                    Vector2d vec_dif = (pt1_w - pt2_w).head(2);
                    double dif = vec_dif.norm();
                    if(dif > 0.2)
                        continue;
                }
                else
                {
                    double dif = std::fabs(pt1_w(2) - pt2_w(2));
                    if(dif > 0.2)
                        continue;
                }

                pt2_find = true;
                ROS_DEBUG("structLineTriangulateByPoints: find pt2 for line-%d: (%lf, %lf, %lf), line type is %d", it_per_id.feature_id, pt2_w(0), pt2_w(1), pt2_w(2), it_per_id.line_type);
                break;
            }
        }
        if(!pt1_find || !pt2_find)
            continue;
        
        it_per_id.line_pluk = getPlukByTwoPoints(pt1_w, pt2_w);
        //initialize param
        Vector3d t_wc = Rs[it_per_id.start_frame] * tic[0] + Ps[it_per_id.start_frame];
        Vector2d line_param = lineParamInitializationByPluk(local_mht, t_wc, it_per_id.line_pluk, it_per_id.line_type);
        ROS_DEBUG("structLineTriangulateByPoints: line-%d param is (%lf, %lf)", it_per_id.feature_id, line_param(0), line_param(1));
        it_per_id.setParam(line_param);
        it_per_id.is_triangulated = true;
        counts++;
    }
    ROS_DEBUG("struct line triangulated by associate points successfully counts: %d", counts);
}

void StructLineFeatureManager::onlyVerticalLineTriangulate(Matrix3d Rs[], Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    int counts = 0;
    for(auto &it_per_id : struct_line_features)
    {
        if(it_per_id.is_triangulated)
            continue;
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if(!(it_per_id.used_num >= line_min_obs && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if(it_per_id.line_type != VERTICAL)
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
            if(imu_j == imu_i)
            {
                pi_i = pointsToPlane(p3_w, p1_w, p2_w);
                n_pi_i = pi_i.head(3).normalized();
                // ROS_DEBUG("line-%d has %d frame", it_per_id.feature_id, it_per_id.used_num);
                // ROS_DEBUG("pi_i:(%lf, %lf, %lf ,%lf)", pi_i[0], pi_i[1], pi_i[2], pi_i[3]);
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
                // ROS_DEBUG("pi_j:(%lf, %lf, %lf, %lf), cosine_dist is %lf", pi_j[0], pi_j[1], pi_j[2], pi_j[3], n_cosine_dist);
            }
        }
        if(min_cosine_dist > line_max_cosine_dist)
            continue;

        it_per_id.line_pluk = planesToLine(pi_i, pi_j);
        //initialize param, the vertical line's mht is always 0°
        Vector3d t_wc = Rs[it_per_id.start_frame] * tic[0] + Ps[it_per_id.start_frame];
        Vector2d line_param = lineParamInitializationByPluk(0, t_wc, it_per_id.line_pluk, it_per_id.line_type);
        it_per_id.setParam(line_param);
        it_per_id.is_triangulated = true;
        counts++;
        ROS_DEBUG("vertical line id-%d triangulate result is (%lf, %lf, %lf, %lf, %lf, %lf)", it_per_id.feature_id, it_per_id.line_pluk[0], it_per_id.line_pluk[1], it_per_id.line_pluk[2], 
                                                                                                          it_per_id.line_pluk[3], it_per_id.line_pluk[4], it_per_id.line_pluk[5]);
    }
    ROS_DEBUG("vertical line triangulate successfully counts: %d", counts);
}

void StructLineFeatureManager::onlyVerticalLineTriangulateByPoints(const FeatureManager &f_manager, Matrix3d Rs[], Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    int counts = 0;
    for(auto &it_per_id : struct_line_features)
    {
        if(it_per_id.is_triangulated || it_per_id.line_type != VERTICAL)
            continue;

        if(it_per_id.associa_points.size() < 2)
            continue;
            
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if(!(it_per_id.used_num >= line_min_obs && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        vector<pair<int, double>> id_pts;
        for(auto it_id_dist : it_per_id.associa_points)
        {
            id_pts.emplace_back(it_id_dist.first, it_id_dist.second);
        }
        sort(id_pts.begin(), id_pts.end(), [](const pair<int, double> &i1, const pair<int, double> &i2){return i1.second < i2.second;});
        //找到满足条件的两个点
        bool pt1_find = false, pt2_find = false;
        Vector3d pt1_w, pt2_w;
        for(auto it_id_dist : id_pts)
        {
            if(!pt1_find)
            {
                int pt1_id = it_id_dist.first;
                auto it = find_if(f_manager.feature.begin(), f_manager.feature.end(), [pt1_id](const FeaturePerId &p){return p.feature_id == pt1_id;});
                if(it == f_manager.feature.end() || it->solve_flag != 1)
                    continue;
                Vector3d pt_uv = it->feature_per_frame[0].point;
                int imu_i = it->start_frame;
                pt1_w = Rs[imu_i]* (ric[0] * (it->estimated_depth * pt_uv) + tic[0]) + Ps[imu_i];
                pt1_find = true;
                ROS_DEBUG("onlyVerticalLineTriangulateByPoints: find pt1 for line-%d: (%lf, %lf, %lf), line type is %d", it_per_id.feature_id, pt1_w(0), pt1_w(1), pt1_w(2), it_per_id.line_type);
                continue;
            }
            if(!pt2_find)
            {
                int pt2_id = it_id_dist.first;
                auto it = find_if(f_manager.feature.begin(), f_manager.feature.end(), [pt2_id](const FeaturePerId &p){return p.feature_id == pt2_id;});
                if(it == f_manager.feature.end() || it->solve_flag != 1)
                    continue;
                Vector3d pt_uv = it->feature_per_frame[0].point;
                int imu_i = it->start_frame;
                pt2_w = Rs[imu_i]* (ric[0] * (it->estimated_depth * pt_uv) + tic[0]) + Ps[imu_i];

                double p1_p2_dist = (pt1_w - pt2_w).norm();
                if(p1_p2_dist < 0.02)
                    continue;

                if(it_per_id.line_type == VERTICAL)
                {
                    Vector2d vec_dif = (pt1_w - pt2_w).head(2);
                    double dif = vec_dif.norm();
                    if(dif > 0.2)
                        continue;
                }
                else
                {
                    double dif = std::fabs(pt1_w(2) - pt2_w(2));
                    if(dif > 0.2)
                        continue;
                }
                
                pt2_find = true;
                ROS_DEBUG("onlyVerticalLineTriangulateByPoints: find pt2 for line-%d: (%lf, %lf, %lf), line type is %d", it_per_id.feature_id, pt2_w(0), pt2_w(1), pt2_w(2), it_per_id.line_type);
                break;
            }
        }
        if(!pt1_find || !pt2_find)
            continue;
        
        it_per_id.line_pluk = getPlukByTwoPoints(pt1_w, pt2_w);
        //initialize param
        Vector3d t_wc = Rs[it_per_id.start_frame] * tic[0] + Ps[it_per_id.start_frame];
        Vector2d line_param = lineParamInitializationByPluk(0, t_wc, it_per_id.line_pluk, it_per_id.line_type);
        ROS_DEBUG("onlyVerticalLineTriangulateByPoints: line-%d param is (%lf, %lf)", it_per_id.feature_id, line_param(0), line_param(1));
        it_per_id.setParam(line_param);
        it_per_id.is_triangulated = true;
        counts++;
    }
    ROS_DEBUG("vertical line triangulated by associate points successfully counts: %d", counts);
}

//初始化已划分线条的两参数(基于pluk方案)
Vector2d StructLineFeatureManager::lineParamInitializationByPluk(double local_mht, const Vector3d &t_ws, const Vector6d &line_w, const LineType &line_type)
{
    Matrix3d R_ws;
    if(line_type == VERTICAL)
        R_ws.setIdentity();
    else
        R_ws << cos(local_mht), -sin(local_mht), 0,
           sin(local_mht), cos(local_mht), 0,
           0, 0, 1;
    Vector6d line_s = plukTransformPose(line_w, R_ws.transpose(), -R_ws.transpose()*t_ws);
    Matrix3d R_sl = getRslByType(line_type);
    Vector6d line_l = plukTransformPose(line_s, R_sl.transpose(), Vector3d(0, 0, 0));
    Vector4d pi_l_xy(0, 0, 1, 0);//plane of xy
    Vector4d point_l = getIntersecByLineAndPlane(pi_l_xy, line_l);
    Vector2d l_param;
    l_param(0) = 1 / (point_l.head(2).norm());//inv_dep
    l_param(1) = atan2(point_l(1), point_l(0));//phi
    return l_param;
}

void StructLineFeatureManager::removeBackShiftParam(Vector3d &marge_P, Vector3d &new_P)
{
    for(auto it = struct_line_features.begin(), it_next = struct_line_features.begin(); it != struct_line_features.end(); it = it_next)
    {
        it_next++;
        if(it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->line_feature_per_frame.erase(it->line_feature_per_frame.begin());
            it->used_num = it->line_feature_per_frame.size();
            if(it->used_num == 0)
                struct_line_features.erase(it);
            else
            {
                if(it->is_triangulated)
                {
                    Vector3d lp_l;
                    double a = 1 / it->inv_depth * cos(it->phi);
                    double b = 1 / it->inv_depth * sin(it->phi);
                    lp_l << a, b, 0;
                    Matrix3d R_sl = getRslByType(it->line_type);
                    Vector3d lp_s = R_sl * lp_l;

                    Matrix3d R_nm;
                    R_nm.setIdentity();
                    Vector3d t_nm = marge_P - new_P;
                    Vector3d lp_s_n = R_nm * lp_s + t_nm;
                    Vector3d lp_l_n = R_sl.transpose() * lp_s_n;
                    it->inv_depth = 1 / (lp_l_n.head(2).norm());
                    it->phi = atan2(lp_l_n(1), lp_l_n(0));
                }
            }
        }
    }
}

void StructLineFeatureManager::removeBack()
{
    for(auto it = struct_line_features.begin(), it_next = struct_line_features.begin(); it != struct_line_features.end(); it = it_next)
    {
        it_next++;
        if(it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->line_feature_per_frame.erase(it->line_feature_per_frame.begin());
            it->used_num = it->line_feature_per_frame.size();
            if(it->used_num == 0)
                struct_line_features.erase(it);
        }
    }
}

void StructLineFeatureManager::removeFront(int frame_count)
{
    for(auto it = struct_line_features.begin(), it_next = struct_line_features.begin(); it != struct_line_features.end(); it = it_next)
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
                struct_line_features.erase(it);
        }
    }
}

int StructLineFeatureManager::getFeatureCount()
{
    int cnt = 0;
    for(auto &it_per_id : struct_line_features)
    {
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if(isLineUsable(it_per_id))
            cnt++;
    }
    return cnt;
}

MatrixXd StructLineFeatureManager::getLineParamMat()
{
    MatrixXd lines_param_mat(getFeatureCount(), 2);
    int index = 0;
    for(auto &it_per_id : struct_line_features)
    {
        if(isLineUsable(it_per_id))
        {
            Vector2d line_param(it_per_id.inv_depth, it_per_id.phi);
            lines_param_mat.row(index++) = line_param;
        }
    }
    return lines_param_mat;
}

MatrixXd StructLineFeatureManager::getLineParamMat(vector<LineType> &lines_type)
{
    lines_type.clear();
    MatrixXd lines_param_mat(getFeatureCount(), 2);
    int index = 0;
    for(auto &it_per_id : struct_line_features)
    {
        if(isLineUsable(it_per_id))
        {
            Vector2d line_param(it_per_id.inv_depth, it_per_id.phi);
            lines_param_mat.row(index++) = line_param;
            lines_type.push_back(it_per_id.line_type);
        }
    }
    return lines_param_mat;
}

void StructLineFeatureManager::setLineFeature(const MatrixXd &lines_param_mat)
{
    int index = 0;
    for(auto &it_per_id : struct_line_features)
    {
        if(isLineUsable(it_per_id))
        {
            Vector2d line_param = lines_param_mat.row(index++);
            it_per_id.inv_depth = line_param[0];
            it_per_id.phi = line_param[1];
        }
    }
}

pair<int, int> StructLineFeatureManager::removeOutlier(set<int> &outlierIndex)
{
    int rm_h_cnt = 0, rm_v_cnt = 0;
    std::set<int>::iterator itSet;
    for (auto it = struct_line_features.begin(), it_next = struct_line_features.begin();
         it != struct_line_features.end(); it = it_next)
    {
        it_next++;
        int index = it->feature_id;
        itSet = outlierIndex.find(index);
        if(itSet != outlierIndex.end())
        {
            if(it->line_type == VERTICAL)
                rm_v_cnt++;
            else if(it->line_type == HORIZON_X || it->line_type == HORIZON_Y)
                rm_h_cnt++;
            struct_line_features.erase(it);
            //printf("remove line outlier %d \n", index);
        }
    }
    return pair<int, int>(rm_v_cnt, rm_h_cnt);
}

pair<int, int> StructLineFeatureManager::getTriangulatedCount()
{
    int all_cnt = 0, tri_cnt = 0;
    for(auto &it_per_id : struct_line_features)
    {
        if(it_per_id.line_type == OTHER)
            continue;
        if(it_per_id.is_triangulated)
            tri_cnt++;
        all_cnt++;
    }
    return pair<int, int>(all_cnt, tri_cnt);
}

void StructLineFeatureManager::getUninitialLines(const map<int, Eigen::Matrix<double, 8, 1>>&cur_lines, vector<pair<int, Vector4d>> &out_lines)
{
    out_lines.clear();
    for(auto &it_per_id : cur_lines)
    {
        int line_id = it_per_id.first;
        auto it = find_if(struct_line_features.begin(), struct_line_features.end(), 
                        [line_id](const StructLineFeaturePerId &l){return l.feature_id == line_id;});
        if(it != struct_line_features.end())
        {
            if(!it->is_triangulated)
                out_lines.emplace_back(line_id, it_per_id.second.head(4));
        }
    }
}

void StructLineFeatureManager::updateLinesAssociaPts(const vector<pair<int, vector<pair<int, double>>>> &lid_associa_pts)
{
    for(auto &id_pts : lid_associa_pts)
    {
        int line_id = id_pts.first;
        auto it = find_if(struct_line_features.begin(), struct_line_features.end(), 
                        [line_id](const StructLineFeaturePerId &l){return l.feature_id == line_id;});
        if(it != struct_line_features.end())
        {
            it->updateAssociaPts(id_pts.second);
        }
    }
}

/*********************************************************MHT Manager*********************************************************/
void MHTManager::clear()
{
    for(int i = 0; i < local_mht_vec.size(); i++)
    {
        local_mht_vec[i] = -1;
    }
}

void MHTManager::slideMHTWindowOld()
{
    for(int i = 0; i < local_mht_vec.size() - 1; i++)
    {
        local_mht_vec[i] = local_mht_vec[i+1];
    }
}

void MHTManager::slideMHTWindowNew()
{
    local_mht_vec[WINDOW_SIZE - 1] = local_mht_vec[WINDOW_SIZE];
}

void MHTManager::insertNewMHT(int frame_count, double new_mht)
{
    local_mht_vec[frame_count] = new_mht;
}

bool MHTManager::checkMHTWindow()
{
    bool is_window_full = true;
    double max_mht = -1;
    double min_mht = 2;
    for(int i = 0; i < local_mht_vec.size(); i++)
    {
        if(local_mht_vec[i] < 0)
        {
            is_window_full = false;
            break;
        }
        if(local_mht_vec[i] > max_mht)
            max_mht = local_mht_vec[i];
        if(local_mht_vec[i] < min_mht)
            min_mht = local_mht_vec[i];
    }
    if(!is_window_full)
        return false;
    if(fabs(max_mht - min_mht) < MHT_NEIGHBOR_DIFF_THRESH)
        return true;
    return false;
}

double MHTManager::getMeanMHT()
{
    double mht_sum = 0;
    for(int i = 0; i < local_mht_vec.size(); i++)
    {
        assert(local_mht_vec[i] >= 0);
        mht_sum += local_mht_vec[i];
    }
    return mht_sum / local_mht_vec.size();
}

void MHTManager::printMHTWindow()
{
    std::ostringstream strs;
    for(int i = 0; i < local_mht_vec.size(); i++)
    {
        strs << local_mht_vec[i] << " ";
    }
    std::string str = strs.str();
    std::cout << "current mht window is [" << str << ']' << std::endl;
}

double MHTManager::getLatestMHT()
{
    if(local_mht_vec.back() != -1)
        return local_mht_vec.back();
    else
        return 0;
}