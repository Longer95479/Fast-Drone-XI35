#pragma once

#include "../estimator/parameters.h"
#include "../utility/utility.h"
#include "../zupt/zupt.h"
#include <Eigen/Dense>
#include <ceres/ceres.h>

class ZuptFactor : public ceres::SizedCostFunction<9, 7, 9> {
  public:
    ZuptFactor(const ZuptResultInfo &zupt_result_info, const Eigen::Vector3d &gravity_global)
        : zupt_info_(zupt_result_info), g_global_(gravity_global) {}

    virtual bool Evaluate(
        double const *const *parameters, double *residuals, double **jacobians) const;

    ZuptResultInfo zupt_info_;
    Eigen::Vector3d g_global_;

    static Eigen::Matrix<double, 9, 9> sqrt_info;
};
