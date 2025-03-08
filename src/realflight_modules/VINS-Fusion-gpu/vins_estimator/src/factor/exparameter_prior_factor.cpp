#include "exparameter_prior_factor.h"

Eigen::Matrix<double, 6, 6> ExParamPriorFactor::sqrt_info;
Eigen::Matrix<double, 3, 3> ExParamPriorFactorOnlyT::sqrt_info;

bool ExParamPriorFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d t_ic(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond q_ic(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    
    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    residual.head<3>() = t_ic - t_pr;
    residual.tail<3>() = 2 * (q_pr.inverse() * q_ic).vec();
    residual = sqrt_info * residual;

    if(jacobians)
    {   
        if(jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 6, 7>> j_tq(jacobians[0]);
            j_tq.leftCols<6>().setIdentity();
            j_tq.rightCols<1>().setZero();
            j_tq = sqrt_info * j_tq;
        }
    }
    return true;
}

bool ExParamPriorFactorOnlyT::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d t_ic(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Map<Eigen::Vector3d> residual(residuals);
    residual = t_ic - t_pr;
    residual = sqrt_info * residual;

    if(jacobians)
    {   
        if(jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 7>> j_tq(jacobians[0]);
            j_tq.setZero();
            j_tq.block<3, 3>(0, 0).setIdentity();
            j_tq = sqrt_info * j_tq;
        }
    }
    return true;
}