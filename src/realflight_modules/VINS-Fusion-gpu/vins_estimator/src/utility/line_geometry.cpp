#include "line_geometry.h"
#include "../sophus/so3.hpp"

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
    double phi = acos(w(0));

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

Matrix3d skewSymmetric(const Vector3d &v) 
{
    Matrix3d S;
    S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return S;
}

//RT-Transform for line_pluk
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
