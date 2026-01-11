#include "zupt_factor.h"

Eigen::Matrix<double, 9, 9> ZuptFactor::sqrt_info;

bool ZuptFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d P(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d V(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Ba(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d Bg(parameters[1][6], parameters[1][7], parameters[1][8]);

    Eigen::Matrix3d R_IG = Q.inverse().toRotationMatrix();
    Eigen::Vector3d am = zupt_info_.acc_raw_;
    Eigen::Vector3d wm = zupt_info_.gyr_raw_;

    Eigen::Map<Eigen::Matrix<double, 9, 1>> residual(residuals);
    residual.block<3, 1>(0, 0) = am + R_IG * g_global_ - Ba;
    residual.block<3, 1>(3, 0) = Bg - wm;
    residual.block<3, 1>(6, 0) = V;

    residual = sqrt_info * residual;

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 9, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
            jacobian_pose.setZero();

            jacobian_pose.block<3, 3>(0, 3) = R_IG * Utility::skewSymmetric(-g_global_);
            jacobian_pose = sqrt_info * jacobian_pose;
        }

        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor>> jacobian_speedbias(jacobians[1]);
            jacobian_speedbias.setZero();

            jacobian_speedbias.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
            jacobian_speedbias.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();
            jacobian_speedbias.block<3, 3>(6, 0) = Eigen::Matrix3d::Identity();
            jacobian_speedbias = sqrt_info * jacobian_speedbias;
        }
    }

    // std::cout << "==============" << std::endl;
    // std::cout << "R_IG: " << std::endl << R_IG << std::endl;
    // std::cout << "resisual: " << std::endl << residual << std::endl;
    // std::cout << "g_global_: " << std::endl << g_global_ << std::endl;
    // std::cout << "am: " << std::endl << am << std::endl;
    // std::cout << "wm: " << std::endl << wm << std::endl;
    // std::cout << "Ba: " << std::endl << Ba << std::endl;
    // std::cout << "Bg: " << std::endl << Bg << std::endl;
    // std::cout << "V: " << std::endl << V << std::endl;
    // std::cout << "==============" << std::endl;

    return true;
}

