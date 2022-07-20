#ifndef ELLIPSOID_FIT_CALIBRATION
#define ELLIPSOID_FIT_CALIBRATION

#include "calibration.h"

namespace imu::calibration::ellipsoid_fit
{
    std::tuple<Offset, RotationMatrix> calibrate(std::unique_ptr<imu::calibration::CalibrationData> &data);
}

#endif