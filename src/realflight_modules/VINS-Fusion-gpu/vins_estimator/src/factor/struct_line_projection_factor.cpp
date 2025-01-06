#include "struct_line_projection_factor.h"

Matrix2d StructLineProjectionOneFrameFactor::sqrt_info;
Matrix2d StructLineProjectionTwoFrameFactor::sqrt_info;

StructLineProjectionOneFrameFactor::StructLineProjectionOneFrameFactor(const Vector3d &_pt_start, const Vector3d &_pt_end, 
    const Vector2d &_pt_velocity_start, const Vector2d &_pt_velocity_end, double _td_i, LineType _line_type): pt_start(_pt_start), pt_end(_pt_end), td_i(_td_i), line_type(_line_type)
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

StructLineProjectionTwoFrameFactor::StructLineProjectionTwoFrameFactor(const Vector3d &_pt_start, const Vector3d &_pt_end, 
    const Vector2d &_pt_velocity_start, const Vector2d &_pt_velocity_end, double _td_i, LineType _line_type): pt_start(_pt_start), pt_end(_pt_end), td_i(_td_i), line_type(_line_type)
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
