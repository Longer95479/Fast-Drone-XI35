#include <eigen3/Eigen/Dense>
#include "struct_line_parameterization.h"

bool StructLineParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    //update inv_depth
    x_plus_delta[0] = x[0] + delta[0];
    //update theta
    double phi = x[1];
    Eigen::Matrix2d SO2_R;
    SO2_R << cos(phi), -sin(phi), sin(phi), cos(phi);
    double delta_phi = delta[1];
    Eigen::Matrix2d delta_R;
    delta_R << cos(delta_phi), -sin(delta_phi), sin(delta_phi), cos(delta_phi);
    SO2_R = SO2_R * delta_R;
    double r11 = SO2_R(0, 0);
    double r21 = SO2_R(1, 0);
    x_plus_delta[1] = atan2(r21, r11);
    return true;
}

bool StructLineParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> j(jacobian);
    j.setIdentity();
    return true;
}