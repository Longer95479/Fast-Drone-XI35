#include "mht_theta_parameterization.h"

bool MHTParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    double theta = x[0];
    Eigen::Matrix2d SO2_R;
    SO2_R  << cos(theta), -sin(theta), sin(theta), cos(theta);
    double delta_theta = delta[0];
    Eigen::Matrix2d SO2_delta_R;
    SO2_R  << cos(delta_theta), -sin(delta_theta), sin(delta_theta), cos(delta_theta);
    SO2_R = SO2_R * SO2_delta_R;
    double r11 = SO2_R(0, 0);
    double r21 = SO2_R(1, 0);
    x_plus_delta[0] = atan2(r21, r11);
    return true;
}

bool MHTParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    jacobian[0] = 1;
    return true;
}