#ifndef LINE_FEATURE_MANAGER__H
#define LINE_FEATURE_MANAGER__H
#include <list>
#include <map>
#include <sstream>
#include <vector>

#include "../utility/line_geometry.h"
#include "feature_manager.h"
#include "parameters.h"
using namespace std;

// this struct is used to record infomation of new lines
struct LineFeats {
  LineFeats(int _id,
            const pair<vector<uchar>, Eigen::Matrix<double, 8, 1>> &line) {
    id = _id;
    line_se = line.second.head(4);
    line_vel = line.second.tail(4);
    lbd_desc = line.first;
    line_type = OTHER;
  }
  void setLineType(LineType type) { line_type = type; }

  static std::shared_ptr<LineFeats> create(
      int _id, const pair<vector<uchar>, Eigen::Matrix<double, 8, 1>> &line) {
    return std::make_shared<LineFeats>(_id, line);
  }

  int id;
  LineType line_type;
  Eigen::Vector4d line_se;
  Eigen::Vector4d line_vel;
  vector<uchar> lbd_desc;
};
typedef std::shared_ptr<LineFeats> LineFeatsPtr;

class LineFeaturePerFrame {
 public:
  LineFeaturePerFrame(const Eigen::Matrix<double, 8, 1> &line_feature,
                      double td) {
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

class LineFeaturePerId {
 public:
  LineFeaturePerId(int feature_id_, int start_frame_)
      : feature_id(feature_id_), start_frame(start_frame_) {
    used_num = 0;
    solve_flag = 0;
    line_pluk.setZero();
  }

  int endFrame() { return start_frame + line_feature_per_frame.size() - 1; }

  void pushFrame(const Eigen::Matrix<double, 8, 1> &line_feature, double td) {
    line_feature_per_frame.emplace_back(line_feature, td);
  }

  void pushFrame(const LineFeatsPtr &line_feature, double td) {
    Eigen::Matrix<double, 8, 1> line_sev;
    line_sev << line_feature->line_se, line_feature->line_vel;
    pushFrame(line_sev, td);
  }

  void updateAssociaPts(const vector<pair<int, double>> &pts_id_dist) {
    for (auto &id_dist : pts_id_dist) {
      associa_points[id_dist.first] = id_dist.second;
    }
  }

  void setDescriptor(const vector<uchar> &desc) { lbd_desc = desc; }

  vector<LineFeaturePerFrame> line_feature_per_frame;

  map<int, double> associa_points;

  int feature_id;
  int start_frame;

  int used_num;
  int solve_flag;  // 0 haven't solve yet; 1 solve succ; 2 solve fail;
  bool is_triangulated = false;

  Vector6d line_pluk;  // in world
  vector<uchar> lbd_desc;
};

class LineFeatureManager {
 public:
  list<LineFeaturePerId> line_features;

  void addLineFeature(
      int frame_cnt,
      const map<int, pair<vector<uchar>, Eigen::Matrix<double, 8, 1>>>
          &img_line,
      double td);
  void line_triangulate(Matrix3d Rs[], Vector3d Ps[], Vector3d tic[],
                        Matrix3d ric[]);
  void removeBack();
  void removeFront(int frame_count);
  int getFeatureCount();
  MatrixXd getLineOrthMat();
  void setLineFeature(const MatrixXd &lineOrthMat);
  void removeOutlier(set<int> &outlierIndex);
  void removeFailures();
};

class StructLineFeaturePerId : public LineFeaturePerId {
 public:
  StructLineFeaturePerId(int feature_id_, int start_frame_, LineType line_type_)
      : LineFeaturePerId(feature_id_, start_frame_), line_type(line_type_) {
    inv_depth = -1;
    phi = 0;
  }

  void setParam(const Vector2d &vec_param) {
    inv_depth = vec_param[0];
    phi = vec_param[1];
  }

  Vector6d getPlukInWorldFromParam(const double local_mht, const Matrix3d Rs[],
                                   const Vector3d Ps[], const Vector3d tic[],
                                   const Matrix3d ric[]) {
    assert(is_triangulated = true);

    Matrix3d R_wi = Rs[start_frame];
    Vector3d t_wi = Ps[start_frame];
    Vector3d t_ws = R_wi * tic[0] + t_wi;
    Matrix3d R_ws;
    if (line_type == VERTICAL)
      R_ws.setIdentity();
    else
      R_ws << cos(local_mht), -sin(local_mht), 0, sin(local_mht),
          cos(local_mht), 0, 0, 0, 1;
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

class StructLineFeatureManager {
 public:
  list<StructLineFeaturePerId> struct_line_features;

  bool isLineUsable(const StructLineFeaturePerId &line);

  void addTrackedStructLine(
      const map<int, pair<vector<uchar>, Eigen::Matrix<double, 8, 1>>>
          &img_line,
      double td, vector<LineFeatsPtr> &new_lines);
  void addTrackedStructLineAndGetHorizon(
      const map<int, pair<vector<uchar>, Eigen::Matrix<double, 8, 1>>>
          &img_line,
      double td, vector<LineFeatsPtr> &h_lines,
      vector<LineFeatsPtr> &new_lines);

  void structLineTriangulate(double local_mht, Matrix3d Rs[], Vector3d Ps[],
                             Vector3d tic[], Matrix3d ric[]);
  void structLineTriangulateByPoints(double local_mht,
                                     const FeatureManager &f_manager,
                                     Matrix3d Rs[], Vector3d Ps[],
                                     Vector3d tic[], Matrix3d ric[]);

  void onlyVerticalLineTriangulate(Matrix3d Rs[], Vector3d Ps[], Vector3d tic[],
                                   Matrix3d ric[]);
  void onlyVerticalLineTriangulateByPoints(const FeatureManager &f_manager,
                                           Matrix3d Rs[], Vector3d Ps[],
                                           Vector3d tic[], Matrix3d ric[]);

  Vector2d lineParamInitializationByPluk(double local_mht, const Vector3d &t_ws,
                                         const Vector6d &line_w,
                                         const LineType &line_type);

  void addNewStrcutLine(int frame_cnt, const vector<LineFeatsPtr> &new_lines,
                        double td);

  void removeBackShiftParam(Vector3d &marge_P, Vector3d &new_P);
  void removeBack();
  void removeFront(int frame_count);

  int getFeatureCount();
  MatrixXd getLineParamMat();
  MatrixXd getLineParamMat(vector<LineType> &lines_type);
  void setLineFeature(const MatrixXd &lines_param_mat);
  void removeFailures();
  pair<int, int> removeOutlier(set<int> &outlierIndex);
  pair<int, int> getTriangulatedCount();

  void getUninitialLines(
      const map<int, pair<vector<uchar>, Eigen::Matrix<double, 8, 1>>>
          &cur_lines,
      vector<pair<int, Vector4d>> &out_lines);
  void updateLinesAssociaPts(
      const vector<pair<int, vector<pair<int, double>>>> &lid_associa_pts);
};

class MHTManager {
 public:
  MHTManager() { local_mht_vec = vector<double>(WINDOW_SIZE + 1, -1); }

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

class GlobalMHTManager {
 public:
  void addLMHT(const vector<double> &mhts) {
    for (double lmht : mhts) global_mhts.push_back(lmht);
  }

  double matchGlobalLMHT(double cur_mht);

  vector<double> global_mhts;
};
#endif