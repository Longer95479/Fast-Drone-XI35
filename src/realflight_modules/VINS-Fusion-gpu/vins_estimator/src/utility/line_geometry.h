#ifndef LINE_GEOMETRY__H
#define LINE_GEOMETRY__H

#include <eigen3/Eigen/Dense>
using namespace Eigen;

using Vector6d = Matrix<double, 6, 1>;
using Matrix6d = Matrix<double, 6, 6>;

Vector4d plukToOrth(const Vector6d &line_pluk);
Vector6d orthToPluk(const Vector4d &line_orth);
Vector4d pointsToPlane(const Vector3d &p1, const Vector3d &p2, const Vector3d &p3);
Vector6d planesToLine(const Vector4d &pi1, const Vector4d &pi2);
Matrix3d skewSymmetric(const Vector3d &v);
Vector6d plukTransformPose(const Vector6d &line_in, const Matrix3d &R, const Vector3d &t);

#endif