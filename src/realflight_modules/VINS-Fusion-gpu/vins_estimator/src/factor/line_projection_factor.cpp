#include "line_projection_factor.h"

Matrix2d LineProjectionFactor::sqrt_info;
double LineProjectionFactor::sum_t;

LineProjectionFactor::LineProjectionFactor(const Vector3d &_pt_start, const Vector3d &_pt_end, 
    const Vector2d &_pt_velocity_start, const Vector2d &_pt_velocity_end, double _td_i)
    : pt_start(_pt_start), pt_end(_pt_end), td_i(_td_i)
{
    pt_velocity_start.x() = _pt_velocity_start.x();
    pt_velocity_start.y() = _pt_velocity_start.y();
    pt_velocity_start.z() = 0;
    pt_velocity_end.x() = _pt_velocity_end.x();
    pt_velocity_end.y() = _pt_velocity_end.y();
    pt_velocity_end.z() = 0;
    assert(pt_start.z() != 0 && pt_end.z() != 0);
    pt_start /= pt_start.z();
    pt_end /= pt_end.z();
}

bool LineProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    
    Vector3d tic(parameters[1][0], parameters[1][1], parameters[1][2]);
    Quaterniond qic(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Vector4d line_w_orth(parameters[2][0],parameters[2][1],parameters[2][2],parameters[2][3]);
    Vector6d line_w_pluk = orthToPluk(line_w_orth);

    double td = parameters[3][0];

    Matrix3d Rwb(Qi);
    Vector3d twb(Pi);
    Matrix3d Rbc(qic);
    Vector3d tbc(tic);

    Vector6d line_b_pluk = plukTransformPose(line_w_pluk, Rwb.transpose(), -Rwb.transpose() * twb);
    Vector6d line_c_pluk = plukTransformPose(line_b_pluk, Rbc.transpose(), -Rbc.transpose() * tbc);
    
    Vector3d nc = line_c_pluk.head(3);
    double ln_square = nc(0) * nc(0) + nc(1) * nc(1);
    double ln_norm = sqrt(ln_square);
    double ln_trinorm = ln_norm * ln_square;

    Vector3d pt_start_td = pt_start - (td - td_i) * pt_velocity_start;
    Vector3d pt_end_td = pt_end - (td - td_i) * pt_velocity_end;

    double e1 = pt_start_td.dot(nc);
    double e2 = pt_end_td.dot(nc);
    Map<Vector2d> residual(residuals);
    residual(0) = e1 / ln_norm;
    residual(1) = e2 / ln_norm;
    residual = sqrt_info * residual;

    if(jacobians)
    {
        //jaco_e_ln
        Matrix<double, 2, 3> jaco_e_ln;
        jaco_e_ln << (pt_start_td(0) / ln_norm - nc(0) * e1 / ln_trinorm), (pt_start_td(1) / ln_norm - nc(1) * e1 / ln_trinorm), 1.0 / ln_norm, 
                     (pt_end_td(0) / ln_norm - nc(0) * e2 / ln_trinorm), (pt_end_td(1) / ln_norm - nc(1) * e2 / ln_trinorm), 1.0 / ln_norm;
        //jaco_ln_lc
        Matrix<double, 3, 6> jaco_ln_lc;
        jaco_ln_lc.setZero();
        jaco_ln_lc.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        Matrix<double, 2, 6> jaco_e_lc;
        jaco_e_lc = sqrt_info * jaco_e_ln * jaco_ln_lc;

        if(jacobians[0])
        {
            Map<Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_e_pose(jacobians[0]);
            
            Matrix6d invTbc;
            invTbc << Rbc.transpose(), -Rbc.transpose()*skewSymmetric(tbc), Eigen::Matrix3d::Zero(),  Rbc.transpose();
            
            Matrix6d jaco_lc_pose; 
            jaco_lc_pose.setZero();
            Vector3d nw = line_w_pluk.head(3);
            Vector3d vw = line_w_pluk.tail(3);
            jaco_lc_pose.block<3, 3>(0, 0) = Rwb.transpose() * skewSymmetric(vw);
            jaco_lc_pose.block<3, 3>(0, 3) = skewSymmetric(Rwb.transpose() * (nw + skewSymmetric(vw) * twb));
            jaco_lc_pose.block<3, 3>(3, 3) = skewSymmetric(Rwb.transpose() * vw);
            jaco_lc_pose = invTbc * jaco_lc_pose;

            jacobian_e_pose.leftCols<6>() = jaco_e_lc * jaco_lc_pose;
            jacobian_e_pose.rightCols<1>().setZero();
        }
        if(jacobians[1])
        {
            Map<Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_e_ex(jacobians[1]);

            Matrix6d jaco_lc_ex;
            jaco_lc_ex.setZero();
            Vector3d nb = line_b_pluk.head(3);
            Vector3d vb = line_b_pluk.tail(3);
            jaco_lc_ex.block<3, 3>(0, 0) = Rbc.transpose() * skewSymmetric(vb);
            jaco_lc_ex.block<3, 3>(0, 3) = skewSymmetric(Rbc.transpose() * (nb + skewSymmetric(vb) * tbc));
            jaco_lc_ex.block<3, 3>(3, 3) = skewSymmetric(Rbc.transpose() * vb);

            jacobian_e_ex.leftCols<6>() = jaco_e_lc * jaco_lc_ex;
            jacobian_e_ex.rightCols<1>().setZero();
        }
        if(jacobians[2])
        {
            Map<Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_e_orth(jacobians[2]);

            Matrix3d Rwc = Rwb * Rbc;
            Vector3d twc = Rwb * tbc + twb;
            Matrix6d invTwc;
            invTwc << Rwc.transpose(), -Rwc.transpose() * skewSymmetric(twc), Eigen::Matrix3d::Zero(), Rwc.transpose();

            Vector3d nw = line_w_pluk.head(3);
            Vector3d vw = line_w_pluk.tail(3);
            Vector3d u1 = nw / nw.norm();
            Vector3d u2 = vw / vw.norm();
            Vector3d u3 = u1.cross(u2);
            Vector2d w(nw.norm(), vw.norm());
            w = w / w.norm();
            double w1 = w(0);
            double w2 = w(1);

            Matrix<double, 6, 4> jaco_lw_orth;
            jaco_lw_orth.setZero();
            jaco_lw_orth.block<3, 1>(0, 1) = -w1 * u3;
            jaco_lw_orth.block<3, 1>(0, 2) = w1 * u2;
            jaco_lw_orth.block<3, 1>(0, 3) = -w2 * u1;
            jaco_lw_orth.block<3, 1>(3, 0) = w2 * u3;
            jaco_lw_orth.block<3, 1>(3, 2) = -w2 * u1;
            jaco_lw_orth.block<3, 1>(3, 3) = w1 * u2;

            jacobian_e_orth = jaco_e_lc * invTwc * jaco_lw_orth;
        }
        if(jacobians[3])
        {
            Map<Vector2d> jacobian_e_td(jacobians[3]);
            jacobian_e_td(0) = -pt_velocity_start.dot(nc) / ln_norm;
            jacobian_e_td(1) = -pt_velocity_end.dot(nc) / ln_norm;
            jacobian_e_td = sqrt_info * jacobian_e_td;
            //jacobian_e_td.setZero();
        }
    }
    return true;
}