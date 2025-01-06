#pragma once
#include <ceres/autodiff_cost_function.h>
#include <eigen3/Eigen/Dense>
#include "../utility/line_geometry.h"

template<typename T>
Eigen::Matrix<T, 3, 3> skewSymmetricTemp(const Eigen::Matrix<T, 3, 1> &v) 
{
    Eigen::Matrix<T, 3, 3>  S;
    S << T(0), -v(2), v(1), v(2), T(0), -v(0), -v(1), v(0), T(0);
    return S;
}

//Rt-Transform for line_pluk
template<typename T>
Eigen::Matrix<T, 6, 1> plukTransformPoseTemp(const Eigen::Matrix<T, 6, 1> &line_in, const Eigen::Matrix<T, 3, 3> &R, const Eigen::Matrix<T, 3, 1> &t)
{
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using Vector6T = Eigen::Matrix<T, 6, 1>;

    Vector3T n_in = line_in.head(3);
    Vector3T v_in = line_in.tail(3);

    Vector3T n_out = R * n_in + skewSymmetricTemp<T>(t) * R * v_in;
    Vector3T v_out = R * v_in;

    Vector6T line_out;
    line_out << n_out, v_out;
    return line_out;
}

class StructLineProjectionOneFrameFactor
{
public:
    StructLineProjectionOneFrameFactor(const Vector3d &_pt_start, const Vector3d &_pt_end, 
                               const Vector2d &_pt_velocity_start, const Vector2d &_pt_velocity_end, 
                               double _td_i, LineType _line_type);

    template<typename T>
    bool operator()(const T *const line_st_ptr, const T *const mht_angle_ptr, const T *const T_wi_ptr, const T *const T_ic_ptr, const T *const td_ptr, T *residual_ptr) const;

    static ceres::CostFunction *create(const Vector3d &_pt_start, const Vector3d &_pt_end, 
                                       const Vector2d &_pt_velocity_start, const Vector2d &_pt_velocity_end, 
                                       double _td_i, LineType _line_type)
    {
        return (new ceres::AutoDiffCostFunction<StructLineProjectionOneFrameFactor, 2, 2, 1, 7, 7, 1>(
            new StructLineProjectionOneFrameFactor(_pt_start, _pt_end, _pt_velocity_start, _pt_velocity_end, 
            _td_i, _line_type)));
    }

    Vector3d pt_start, pt_end;
    Vector3d pt_velocity_start, pt_velocity_end;
    double td_i;
    LineType line_type;
    static Matrix2d sqrt_info;
};

template<typename T>
bool StructLineProjectionOneFrameFactor::operator()(const T *const line_st_ptr, const T *const mht_angle_ptr, const T *const T_wi_ptr, const T *const T_ic_ptr, const T *const td_ptr, T *residual_ptr) const
{
    using Vector2T = Eigen::Matrix<T, 2, 1>;
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using Vector6T = Eigen::Matrix<T, 6, 1>;
    using Matrix3T = Eigen::Matrix<T, 3, 3>;
    using QuaternionT = Eigen::Quaternion<T>;
    
    //line in L
    T inv_depth = line_st_ptr[0];
    T theta = line_st_ptr[1];
    assert(inv_depth != T(0));
    T a = T(1) / inv_depth * ceres::cos(theta);
    T b = T(1) / inv_depth * ceres::sin(theta);
    Vector6T l_L_pluk;
    l_L_pluk << b, -a, T(0), T(0), T(0), T(1);

    Matrix3T R_sl;
    switch (line_type)
    {
    case VERTICAL:
        R_sl.setIdentity();
        break;
    case HORIZON_X:
        R_sl << T(0), T(0), T(1), T(0), T(1), T(0), T(-1), T(0), T(0);
        break;
    case HORIZON_Y:
        R_sl << T(1), T(0), T(0), T(0), T(0), T(1), T(0), T(-1), T(0);
        break;
    default:
        return false;
    }

    T m_angle = mht_angle_ptr[0];
    Matrix3T R_ws;
    if(line_type == VERTICAL)
        R_ws.setIdentity();
    else
        R_ws << ceres::cos(m_angle), -ceres::sin(m_angle), T(0), 
                ceres::sin(m_angle), ceres::cos(m_angle), T(0),
                T(0), T(0), T(1);

    Vector3T t_wi(T_wi_ptr[0], T_wi_ptr[1], T_wi_ptr[2]);
    QuaternionT Q_wi(T_wi_ptr[6], T_wi_ptr[3], T_wi_ptr[4], T_wi_ptr[5]);
    Matrix3T R_wi(Q_wi);

    Vector3T t_ic(T_ic_ptr[0], T_ic_ptr[1], T_ic_ptr[2]);
    QuaternionT Q_ic(T_ic_ptr[6], T_ic_ptr[3], T_ic_ptr[4], T_ic_ptr[5]);
    Matrix3T R_ic(Q_ic);

    Vector3T t_ws = R_wi * t_ic + t_wi;

    T td = td_ptr[0];

    Vector6T l_s_pluk = plukTransformPoseTemp<T>(l_L_pluk, R_sl, Vector3T(T(0), T(0), T(0)));
    Vector6T l_w_pluk = plukTransformPoseTemp<T>(l_s_pluk, R_ws, t_ws);
    Vector6T l_i_pluk = plukTransformPoseTemp<T>(l_w_pluk, R_wi.transpose(), -R_wi.transpose()*t_wi);
    Vector6T l_c_pluk = plukTransformPoseTemp<T>(l_i_pluk, R_ic.transpose(), -R_ic.transpose()*t_ic);
    
    Vector3T nc = l_c_pluk.head(3);
    T ln_square = nc(0) * nc(0) + nc(1) * nc(1);
    T ln_norm = ceres::sqrt(ln_square);

    Vector3T pt_start_td = pt_start.cast<T>() - (td - T(td_i)) * pt_velocity_start.cast<T>();
    Vector3T pt_end_td = pt_end.cast<T>() - (td - T(td_i)) * pt_velocity_end.cast<T>();

    T e1 = pt_start_td.dot(nc);
    T e2 = pt_end_td.dot(nc);
    Eigen::Map<Vector2T> residual(residual_ptr);
    residual(0) = e1 / ln_norm;
    residual(1) = e2 / ln_norm;
    residual = sqrt_info.cast<T>() * residual;

    return true;
}

/****************************************************************************************************************************************************************/
class StructLineProjectionTwoFrameFactor
{
public:
    StructLineProjectionTwoFrameFactor(const Vector3d &_pt_start, const Vector3d &_pt_end, 
                               const Vector2d &_pt_velocity_start, const Vector2d &_pt_velocity_end, 
                               double _td_i, LineType _line_type);

    template<typename T>
    bool operator()(const T *const line_st_ptr, const T *const mht_angle_ptr, const T *const T_wi_ptr, const T *const T_wj_ptr, const T *const T_ic_ptr, const T *const td_ptr, T *residual_ptr) const;

    static ceres::CostFunction *create(const Vector3d &_pt_start, const Vector3d &_pt_end, 
                                       const Vector2d &_pt_velocity_start, const Vector2d &_pt_velocity_end, 
                                       double _td_i, LineType _line_type)
    {
        return (new ceres::AutoDiffCostFunction<StructLineProjectionTwoFrameFactor, 2, 2, 1, 7, 7, 7, 1>(
            new StructLineProjectionTwoFrameFactor(_pt_start, _pt_end, _pt_velocity_start, _pt_velocity_end, 
            _td_i, _line_type)));
    }

    Vector3d pt_start, pt_end;
    Vector3d pt_velocity_start, pt_velocity_end;
    double td_i;
    LineType line_type;
    static Matrix2d sqrt_info;
};

template<typename T>
bool StructLineProjectionTwoFrameFactor::operator()(const T *const line_st_ptr, const T *const mht_angle_ptr, const T *const T_wi_ptr, const T *const T_wj_ptr, const T *const T_ic_ptr, const T *const td_ptr, T *residual_ptr) const
{    
    using Vector2T = Eigen::Matrix<T, 2, 1>;
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using Vector6T = Eigen::Matrix<T, 6, 1>;
    using Matrix3T = Eigen::Matrix<T, 3, 3>;
    using QuaternionT = Eigen::Quaternion<T>;
    //line in L
    T inv_depth = line_st_ptr[0];
    T theta = line_st_ptr[1];
    assert(inv_depth != T(0));
    T a = T(1) / inv_depth * ceres::cos(theta);
    T b = T(1) / inv_depth * ceres::sin(theta);
    Vector6T l_L_pluk;
    l_L_pluk << b, -a, T(0), T(0), T(0), T(1);

    Matrix3T R_sl;
    switch (line_type)
    {
    case VERTICAL:
        R_sl.setIdentity();
        break;
    case HORIZON_X:
        R_sl << T(0), T(0), T(1), T(0), T(1), T(0), T(-1), T(0), T(0);
        break;
    case HORIZON_Y:
        R_sl << T(1), T(0), T(0), T(0), T(0), T(1), T(0), T(-1), T(0);
        break;
    default:
        return false;
    }

    T m_angle = mht_angle_ptr[0];
    Matrix3T R_ws;
    if(line_type == VERTICAL)
        R_ws.setIdentity();
    else
        R_ws << ceres::cos(m_angle), -ceres::sin(m_angle), T(0), 
                ceres::sin(m_angle), ceres::cos(m_angle), T(0),
                T(0), T(0), T(1);

    Vector3T t_wi(T_wi_ptr[0], T_wi_ptr[1], T_wi_ptr[2]);
    QuaternionT Q_wi(T_wi_ptr[6], T_wi_ptr[3], T_wi_ptr[4], T_wi_ptr[5]);
    Matrix3T R_wi(Q_wi);

    Vector3T t_wj(T_wj_ptr[0], T_wj_ptr[1], T_wj_ptr[2]);
    QuaternionT Q_wj(T_wj_ptr[6], T_wj_ptr[3], T_wj_ptr[4], T_wj_ptr[5]);
    Matrix3T R_wj(Q_wj);

    Vector3T t_ic(T_ic_ptr[0], T_ic_ptr[1], T_ic_ptr[2]);
    QuaternionT Q_ic(T_ic_ptr[6], T_ic_ptr[3], T_ic_ptr[4], T_ic_ptr[5]);
    Matrix3T R_ic(Q_ic);

    Vector3T t_ws = R_wi * t_ic + t_wi;

    T td = td_ptr[0];

    Vector6T l_s_pluk = plukTransformPoseTemp<T>(l_L_pluk, R_sl, Vector3T(T(0), T(0), T(0)));
    Vector6T l_w_pluk = plukTransformPoseTemp<T>(l_s_pluk, R_ws, t_ws);
    Vector6T l_j_pluk = plukTransformPoseTemp<T>(l_w_pluk, R_wj.transpose(), -R_wj.transpose()*t_wj);
    Vector6T l_c_pluk = plukTransformPoseTemp<T>(l_j_pluk, R_ic.transpose(), -R_ic.transpose()*t_ic);
    
    Vector3T nc = l_c_pluk.head(3);
    T ln_square = nc(0) * nc(0) + nc(1) * nc(1);
    T ln_norm = ceres::sqrt(ln_square);

    Vector3T pt_start_td = pt_start.cast<T>() - (td - T(td_i)) * pt_velocity_start.cast<T>();
    Vector3T pt_end_td = pt_end.cast<T>() - (td - T(td_i)) * pt_velocity_end.cast<T>();

    T e1 = pt_start_td.dot(nc);
    T e2 = pt_end_td.dot(nc);
    Eigen::Map<Vector2T> residual(residual_ptr);
    residual(0) = e1 / ln_norm;
    residual(1) = e2 / ln_norm;
    residual = sqrt_info.cast<T>() * residual;

    return true;
}
