#include "calibration.h"

using namespace imu::calibration;

imu::Coordinates<double> imu::calibration::correctCoordinates(imu::Coordinates<int16_t> &coordinates,
                                                              const Offset &offset,
                                                              const RotationMatrix &rot)
{
    imu::Coordinates<double> corrected;

    corrected.x = coordinates.x + offset.x;
    corrected.y = coordinates.y + offset.y;
    corrected.z = coordinates.z + offset.z;

    double x_ = rot.m00 * corrected.x + rot.m01 * corrected.y + rot.m02 * corrected.z;
    double y_ = rot.m10 * corrected.x + rot.m11 * corrected.y + rot.m12 * corrected.z;
    double z_ = rot.m20 * corrected.x + rot.m21 * corrected.y + rot.m22 * corrected.z;

    corrected.x = x_;
    corrected.y = y_;
    corrected.z = z_;

    return corrected;
}