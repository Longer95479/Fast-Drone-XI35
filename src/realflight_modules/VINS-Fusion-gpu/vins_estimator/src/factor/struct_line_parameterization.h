#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

class StructLineParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const override;
    virtual int GlobalSize() const { return 2; };
    virtual int LocalSize() const { return 2; };
};