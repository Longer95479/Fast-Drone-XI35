#ifndef LINE_FEATURE_MANAGER__H
#define LINE_FEATURE_MANAGER__H
#include "../utility/line_geometry.h"
#include "parameters.h"
#include <sstream>
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

    void updateAssociaPts(const vector<pair<int, double>> &pts_id_dist)
    {
        for(auto &id_dist : pts_id_dist)
        {
            associa_points[id_dist.first] = id_dist.second;
        }
    }

    vector<LineFeaturePerFrame> line_feature_per_frame;

    map<int, double> associa_points;

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

class StructLineFeaturePerId : public LineFeaturePerId
{
public:
    StructLineFeaturePerId(int feature_id_, int start_frame_, LineType line_type_): LineFeaturePerId(feature_id_, start_frame_), line_type(line_type_) {}

    void setParam(const Vector2d &vec_param)
    {
        inv_depth = vec_param[0];
        phi = vec_param[1];
    }

    Vector6d getPlukInWorldFromParam(const double local_mht, const Matrix3d Rs[], const Vector3d Ps[], const Vector3d tic[], const Matrix3d ric[])
    {
        assert(is_triangulated = true);

        Matrix3d R_wi = Rs[start_frame];
        Vector3d t_wi = Ps[start_frame];
        Vector3d t_ws = R_wi * tic[0] + t_wi;
        Matrix3d R_ws;
        if(line_type == VERTICAL)
            R_ws.setIdentity();
        else
            R_ws << cos(local_mht), -sin(local_mht), 0, 
                    sin(local_mht), cos(local_mht), 0,
                    0, 0, 1;
        Matrix3d R_sl = getRslByType(line_type);
        Vector6d pluk_l = getPlukInLocalFromParam(inv_depth, phi);
        Vector6d pluk_s = plukTransformPose(pluk_l, R_sl, Vector3d(0, 0, 0));
        Vector6d pluk_w = plukTransformPose(pluk_s, R_ws, t_ws);
        
        line_pluk = pluk_w;
        return line_pluk;
    }

    LineType line_type;
    double inv_depth;
    double phi;
};

class StructLineFeatureManager
{
public:
    list<StructLineFeaturePerId> struct_line_features;

    bool isLineUsable(const StructLineFeaturePerId& line);
    void addTrackedStructLine(const map<int, Eigen::Matrix<double, 8, 1>> &img_line, double td, vector<pair<int, Eigen::Matrix<double, 8, 1>>> &new_lines);
    void addTrackedStructLineAndGetHorizon(const map<int, Eigen::Matrix<double, 8, 1>> &img_line, double td, vector<pair<int, Eigen::Matrix<double, 8, 1>>> &h_lines, vector<pair<int, Eigen::Matrix<double, 8, 1>>> &new_lines);
    void structLineTriangulate(double local_mht, Matrix3d Rs[], Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void onlyVerticalLineTriangulate(Matrix3d Rs[], Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    Vector2d lineParamInitializationByPluk(double local_mht, const Vector3d &t_ws, const Vector6d &line_w, const LineType &line_type);
    void addNewStrcutLine(int frame_cnt, const vector<pair<int, Eigen::Matrix<double, 8, 1>>> &new_lines, 
                        const vector<LineType> &lines_type, double td);
    void removeBackShiftParam(Vector3d &marge_P, Vector3d &new_P);
    void removeBack();
    void removeFront(int frame_count);
    int getFeatureCount();
    MatrixXd getLineParamMat();
    MatrixXd getLineParamMat(vector<LineType> &lines_type);
    void setLineFeature(const MatrixXd &lines_param_mat);
    pair<int, int> removeOutlier(set<int> &outlierIndex);
    pair<int, int> getTriangulatedCount();

    void getUninitialLines(const map<int, Eigen::Matrix<double, 8, 1>>&cur_lines, vector<pair<int, Vector4d>> &out_lines);
};

class MHTManager
{
public:
    MHTManager()
    {
        local_mht_vec = vector<double>(WINDOW_SIZE + 1, -1);
    }

    void clear();
    void slideMHTWindowOld();
    void slideMHTWindowNew();
    void insertNewMHT(int frame_count, double new_mht);
    bool checkMHTWindow();
    double getMeanMHT();
    void printMHTWindow();
    double getLatestMHT();

    vector<double> local_mht_vec;
};
#endif