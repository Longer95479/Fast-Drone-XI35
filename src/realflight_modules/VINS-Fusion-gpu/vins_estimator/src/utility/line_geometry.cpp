#include "line_geometry.h"
#include "../sophus/so3.hpp"
#include <ros/ros.h>

Matrix3d skewSymmetric(const Vector3d &v) 
{
    Matrix3d S;
    S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return S;
}

//pluk to orth
Vector4d plukToOrth(const Vector6d &line_pluk)
{
    Vector3d n = line_pluk.head(3);
    Vector3d v = line_pluk.tail(3);
    Matrix3d n_R;
    n_R.col(0) = n / n.norm();
    n_R.col(1) = v / v.norm();
    n_R.col(2) = n.cross(v) / (n.cross(v).norm());
    Sophus::SO3d n_SO3(n_R);
    Vector3d n_so3 = n_SO3.log();

    Vector2d w(n.norm(), v.norm());
    w = w / w.norm();
    //double phi = acos(w(0));
    double phi = atan2(w(1), w(0));
    
    Vector4d line_orth;
    line_orth.head(3) = n_so3;
    line_orth(3) = phi;
    return line_orth;
}

//orth to pluk
Vector6d orthToPluk(const Vector4d &line_orth)
{
    Vector3d n_so3 = line_orth.head(3);
    Sophus::SO3d n_SO3 = Sophus::SO3d::exp(n_so3);
    Matrix3d n_R = n_SO3.matrix();

    Vector3d u1 = n_R.col(0);
    Vector3d u2 = n_R.col(1);

    double w1 = cos(line_orth(3));
    double w2 = sin(line_orth(3));

    Vector6d line_pluk;
    line_pluk.head(3) = w1 * u1;
    line_pluk.tail(3) = w2 * u2;
    return line_pluk;
}

//three points form a plane [n,w]
Vector4d pointsToPlane(const Vector3d &p1, const Vector3d &p2, const Vector3d &p3)
{
    Vector4d pi;
    pi << (p2 - p1).cross(p3 - p1), -p1.dot(p2.cross(p3));
    return pi;
}

//two planes form a pluk line
Vector6d planesToLine(const Vector4d &pi1, const Vector4d &pi2)
{
    Matrix4d l_mat = pi1 * pi2.transpose() - pi2 * pi1.transpose();
    Vector6d line_pluk;
    line_pluk << l_mat(0, 3), l_mat(1, 3), l_mat(2, 3), -l_mat(1, 2), l_mat(0, 2), -l_mat(0, 1);
    return line_pluk;
}



//Rt-Transform for line_pluk
Vector6d plukTransformPose(const Vector6d &line_in, const Matrix3d &R, const Vector3d &t)
{
    Vector3d n_in = line_in.head(3);
    Vector3d v_in = line_in.tail(3);

    Vector3d n_out = R * n_in + skewSymmetric(t) * R * v_in;
    Vector3d v_out = R * v_in;

    Vector6d line_out;
    line_out << n_out, v_out;
    return line_out;
}

//get abs angle differ between two lines
double getTwoLinesAbsAngle(const Vector4d &line0, const Vector4d &line1)
{
    Vector2d l0_vec(line0[2] - line0[0], line0[3] - line0[1]);
    Vector2d l1_vec(line1[2] - line1[0], line1[3] - line1[1]);
    l0_vec.normalize();
    l1_vec.normalize();
    return acos(fabs(l0_vec.dot(l1_vec)));
}

//returns the average distance between the two endpoints of line1 and line0
double getTwoLinesDistByP2L(const Vector4d &line0, const Vector4d &line1)
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
//get the vanishing point based on the DD in camera unit sphere coordinate system 
Vector2d getVpFromDDs(const Vector3d &DD)
{
    double scalar;
    if(fabs(DD[2]) < 1e-6)
        scalar = DD[2] < 0 ? -1e6 : 1e6;
    else
        scalar = 1 / DD[2];
    Vector2d vp = DD.head(2);
    vp *= scalar;
    return vp;
}
//get pluk matrix (order: [x,y,z,w])
Matrix4d getLMatrixFromPluk(const Vector6d &line_pluk)
{
    Vector3d n = line_pluk.head(3);
    Vector3d v = line_pluk.tail(3);
    Matrix4d pluk_mat;
    pluk_mat.setZero();
    pluk_mat.block<3, 3>(0, 0) = skewSymmetric(n);
    pluk_mat.block<3, 1>(0, 3) = v;
    pluk_mat.block<1, 3>(3, 0) = -v.transpose();
    return pluk_mat;
}
//get dual-pluk matrix
Matrix4d getLDualMatrixFromPluk(const Vector6d &line_pluk)
{
    Vector3d n = line_pluk.head(3);
    Vector3d v = line_pluk.tail(3);
    Matrix4d pluk_mat;
    pluk_mat.setZero();
    pluk_mat.block<3, 3>(0, 0) = skewSymmetric(-v);
    pluk_mat.block<3, 1>(0, 3) = n;
    pluk_mat.block<1, 3>(3, 0) = -n.transpose();
    return pluk_mat;
}
// get Rsl accroding to a line type
Matrix3d getRslByType(const LineType &type)
{
    Matrix3d R_sl;
    switch (type)
    {
    case VERTICAL:
        R_sl.setIdentity();
        break;
    case HORIZON_X:
        R_sl << 0, 0, 1, 0, 1, 0, -1, 0, 0;
        break;
    case HORIZON_Y:
        R_sl << 1, 0, 0, 0, 0, 1, 0, -1, 0;
        break;
    default:
        break;
    }
    return R_sl;
}
//get the pluk from param at local coordinate
Vector6d getPlukInLocalFromParam(double inv_depth, double theta)
{
    double a = 1 / inv_depth * cos(theta);
    double b = 1 / inv_depth * sin(theta);
    Vector6d pluk_l;
    pluk_l << b, -a, 0, 0, 0, 1;
    return pluk_l;
}
// get the intersection of a line and a plane
Vector4d getIntersecByLineAndPlane(const Vector4d &pi, const Vector6d &line)
{
    Matrix4d plukMat = getLMatrixFromPluk(line);
    Vector4d point = plukMat * pi;
    assert(point(3) != 0);
    point /= point(3);
    return point;
}
//get line's 3-parameter expression at 2d-plane 
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
//get intersection between two lines at 2d-plane
Vector2d getIntersecByTwoLine(const Vector3d &l0, const Vector3d &l1)
{
    Matrix2d A;
    Vector2d b;
    A << l0(0), l0(1), l1(0), l1(1);
    b << -l0(2), -l1(2);
    Vector2d x = A.ldlt().solve(b);
    return x;
}