#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <memory>
#include <condition_variable>

#include "imu.h"
#include "i2c_imu_device.h"
#include "../calibration/calibration.h"

namespace imu
{
    struct ImuSensor
    {
        void init()
        {
            sensor->init();
        }

        void start()
        {
            sensor->start();
        }

        void finish()
        {
            sensor->finish();
        }

        std::shared_ptr<I2cIMUDevice> sensor = nullptr;
        imu::Coordinates<int16_t> coordinates;
        std::mutex mtx;
        std::condition_variable dataReadyCV;
        std::unique_ptr<imu::calibration::CalibrationData> calibrationData = nullptr;
        bool isCalibrated = false;
        imu::calibration::Offset offset;
        imu::calibration::RotationMatrix rot;
    };
}

#endif // IMU_SENSOR