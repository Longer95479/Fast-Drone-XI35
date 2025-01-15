#pragma once
#include <eigen3/Eigen/Dense>
using namespace Eigen;

typedef enum
{
    OTHER,
    VERTICAL,  //xy-plane
    HORIZON_X, //yz-plane
    HORIZON_Y  //xz-plane
}LineType;

using Vector6d = Matrix<double, 6, 1>;
using Matrix6d = Matrix<double, 6, 6>;

template<typename T>
Eigen::Matrix<T, 3, 3> skewSymmetricTemp(const Eigen::Matrix<T, 3, 1> &v) 
{
    Eigen::Matrix<T, 3, 3>  S;
    S << T(0), -v(2), v(1), v(2), T(0), -v(0), -v(1), v(0), T(0);
    return S;
}

//Rt-Transform for line_pluk
template<typename T>
Eigen::Matrix<T, 6, 1> plukTransformPoseTemp(const Eigen::Matrix<T, 6, 1> &line_in, const Eigen::Matrix<T, 3, 3> &R, const Eigen::Matrix<T, 3, 1> &t)
{
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using Vector6T = Eigen::Matrix<T, 6, 1>;

    Vector3T n_in = line_in.head(3);
    Vector3T v_in = line_in.tail(3);

    Vector3T n_out = R * n_in + skewSymmetricTemp<T>(t) * R * v_in;
    Vector3T v_out = R * v_in;

    Vector6T line_out;
    line_out << n_out, v_out;
    return line_out;
}

Vector4d plukToOrth(const Vector6d &line_pluk);
Vector6d orthToPluk(const Vector4d &line_orth);
Vector4d pointsToPlane(const Vector3d &p1, const Vector3d &p2, const Vector3d &p3);
Vector6d planesToLine(const Vector4d &pi1, const Vector4d &pi2);
Matrix3d skewSymmetric(const Vector3d &v);
Vector6d plukTransformPose(const Vector6d &line_in, const Matrix3d &R, const Vector3d &t);
double getTwoLinesAbsAngle(const Vector4d &line0, const Vector4d &line1);
double getTwoLinesDistByP2L(const Vector4d &line0, const Vector4d &line1);
Vector2d getVpFromDDs(const Vector3d &DD);
Matrix4d getLMatrixFromPluk(const Vector6d &line_pluk);
Matrix4d getLDualMatrixFromPluk(const Vector6d &line_pluk);
Matrix3d getRslByType(const LineType &type);
Vector6d getPlukInLocalFromParam(double inv_depth, double theta);
Vector4d getIntersecByLineAndPlane(const Vector4d &pi, const Vector6d &line);
Vector3d getLineExpression(const Vector4d &line);
Vector2d getIntersecByTwoLine(const Vector3d &l0, const Vector3d &l1);
double normalizeAngle(double angle);
Vector6d getPlukByTwoPoints(const Vector3d &pt1, const Vector3d &pt2);
