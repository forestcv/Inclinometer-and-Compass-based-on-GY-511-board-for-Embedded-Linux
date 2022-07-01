#ifndef MIN_MAX_CALIBRATION_H
#define MIN_MAX_CALIBRATION_H

#include "calibration.h"
#include "../imu/i2c_imu_device.h"

namespace imu::calibration::min_max_calibration
{
    Offset calibrate(std::unique_ptr<imu::calibration::CalibrationData> &data);
};

#endif // MIN_MAX_CALIBRATION_H