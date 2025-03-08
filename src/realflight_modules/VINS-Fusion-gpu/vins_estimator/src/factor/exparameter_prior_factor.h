#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../estimator/parameters.h"

class ExParamPriorFactor : public ceres::SizedCostFunction<6, 7>
{
public:
    ExParamPriorFactor(const Eigen::Quaterniond &_q_pr, const Eigen::Vector3d &_t_pr) : q_pr(_q_pr), t_pr(_t_pr){}
    ExParamPriorFactor(const Eigen::Matrix3d &_r_pr, const Eigen::Vector3d &_t_pr) : q_pr(Eigen::Quaterniond(_r_pr)), t_pr(_t_pr){}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Quaterniond q_pr;
    Eigen::Vector3d t_pr;
    static Eigen::Matrix<double, 6, 6> sqrt_info;
};

class ExParamPriorFactorOnlyT : public ceres::SizedCostFunction<3, 7>
{
public:
    ExParamPriorFactorOnlyT(const Eigen::Vector3d &_t_pr) : t_pr(_t_pr){}
    
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Vector3d t_pr;
    static Eigen::Matrix<double, 3, 3> sqrt_info;
};