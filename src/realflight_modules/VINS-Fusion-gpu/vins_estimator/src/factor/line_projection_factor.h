#pragma once
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/line_geometry.h"

class LineProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 4, 1>
{
public:
    LineProjectionFactor(const Vector3d &_pt_start, const Vector3d &_pt_end, 
                        const Vector2d &_pt_velocity_start, const Vector2d &_pt_velocity_end, double _td_i);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;
    Vector3d pt_start, pt_end;
    Vector3d pt_velocity_start, pt_velocity_end;
    double td_i;
    static Matrix2d sqrt_info;
    static double sum_t;
};