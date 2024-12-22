#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

class LineOrthParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const override;
    virtual int GlobalSize() const override {return 4;};
    virtual int LocalSize() const override {return 4;};
};
