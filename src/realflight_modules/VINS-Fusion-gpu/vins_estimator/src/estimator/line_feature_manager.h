#ifndef LINE_FEATURE_MANAGER__H
#define LINE_FEATURE_MANAGER__H
#include "../utility/line_geometry.h"
#include "parameters.h"
#include <vector>
#include <list>
#include <map>
using namespace std;

class LineFeaturePerFrame
{
public:
    LineFeaturePerFrame(const Eigen::Matrix<double, 8, 1> &line_feature, double td)
    {
        pt_start.x() = line_feature(0);
        pt_start.y() = line_feature(1);
        pt_start.z() = 1.0;
        pt_end.x() = line_feature(2);
        pt_end.y() = line_feature(3);
        pt_end.z() = 1.0;
        velocity_start.x() = line_feature(4);
        velocity_start.y() = line_feature(5);
        velocity_end.x() = line_feature(6);
        velocity_end.y() = line_feature(7);
        cur_td = td;
    }

    double cur_td;
    Vector3d pt_start;
    Vector3d pt_end;
    Vector2d velocity_start;
    Vector2d velocity_end;
};

class LineFeaturePerId
{
public:
    LineFeaturePerId(int feature_id_, int start_frame_):feature_id(feature_id_), start_frame(start_frame_)
    {
    }

    int endFrame()
    {
        return start_frame + line_feature_per_frame.size() - 1;
    }

    void pushFrame(const Eigen::Matrix<double, 8, 1> &line_feature, double td)
    {
        line_feature_per_frame.emplace_back(line_feature, td);
    }

    vector<LineFeaturePerFrame> line_feature_per_frame;

    int feature_id;
    int start_frame;

    int used_num;
    int solve_flag;
    bool is_triangulated = false;

    Vector6d line_pluk;//in world
};

class LineFeatureManager
{
public:

    list<LineFeaturePerId> line_features;

    void addLineFeature(int frame_cnt, const map<int, Eigen::Matrix<double, 8, 1>> &img_line, double td);
    void line_triangulate(Matrix3d Rs[], Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void removeBack();
    void removeFront(int frame_count);
    int getFeatureCount();
    MatrixXd getLineOrthMat();
    void setLineFeature(const MatrixXd &lineOrthMat);
    void removeOutlier(set<int> &outlierIndex);
};



#endif