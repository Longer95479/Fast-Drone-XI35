#include "vertical_line_projection_factor.h"

Matrix2d VerticalLineProjectionOneFrameFactor::sqrt_info;
Matrix2d VerticalLineProjectionTwoFrameFactor::sqrt_info;

VerticalLineProjectionOneFrameFactor::VerticalLineProjectionOneFrameFactor(const Vector3d &_pt_start, const Vector3d &_pt_end): 
    pt_start(_pt_start), pt_end(_pt_end)
{
    assert(pt_start.z() != 0 && pt_end.z() != 0);
    pt_start /= pt_start.z();
    pt_end /= pt_end.z();
}

VerticalLineProjectionTwoFrameFactor::VerticalLineProjectionTwoFrameFactor(const Vector3d &_pt_start, const Vector3d &_pt_end): 
    pt_start(_pt_start), pt_end(_pt_end)
{
    assert(pt_start.z() != 0 && pt_end.z() != 0);
    pt_start /= pt_start.z();
    pt_end /= pt_end.z();
}