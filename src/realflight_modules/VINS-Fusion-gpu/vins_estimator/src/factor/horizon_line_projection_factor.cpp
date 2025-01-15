#include "horizon_line_projection_factor.h"

Matrix2d HorizonLineProjectionOneFrameFactor::sqrt_info;
Matrix2d HorizonLineProjectionTwoFrameFactor::sqrt_info;

HorizonLineProjectionOneFrameFactor::HorizonLineProjectionOneFrameFactor(const Vector3d &_pt_start, const Vector3d &_pt_end, LineType _horizon_line_type): 
    pt_start(_pt_start), pt_end(_pt_end), horizon_line_type(_horizon_line_type)
{
    assert(pt_start.z() != 0 && pt_end.z() != 0);
    pt_start /= pt_start.z();
    pt_end /= pt_end.z();
}

HorizonLineProjectionTwoFrameFactor::HorizonLineProjectionTwoFrameFactor(const Vector3d &_pt_start, const Vector3d &_pt_end, LineType _horizon_line_type): 
    pt_start(_pt_start), pt_end(_pt_end), horizon_line_type(_horizon_line_type)
{
    assert(pt_start.z() != 0 && pt_end.z() != 0);
    pt_start /= pt_start.z();
    pt_end /= pt_end.z();
}