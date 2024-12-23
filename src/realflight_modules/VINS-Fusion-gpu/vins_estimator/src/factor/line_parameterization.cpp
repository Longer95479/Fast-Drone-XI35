#include "line_parameterization.h"
#include "../sophus/so3.hpp"

bool LineOrthParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    //update SO3
    Eigen::Map<const Eigen::Vector3d> x_so3(x);
    Eigen::Map<const Eigen::Vector3d> update_so3(delta);
    Sophus::SO3d x_SO3 = Sophus::SO3d::exp(x_so3);
    x_SO3 =  Sophus::SO3d::exp(update_so3) * x_SO3; //左扰动
    Eigen::Map<Eigen::Vector3d> so3_updated(x_plus_delta);
    so3_updated = x_SO3.log();

    //update SO2
    double phi = x[3];
    Eigen::Matrix<double, 2, 2> w, delta_w;
    w << cos(phi), -sin(phi), sin(phi), cos(phi);
    double delta_phi = delta[3];
    delta_w << cos(delta_phi), -sin(delta_phi), sin(delta_phi), cos(delta_phi);
    w = w * delta_w;
    x_plus_delta[3] = acos(w(0, 0));
    return true;
}

bool LineOrthParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> j(jacobian);
    j.setIdentity();
    return true;
}