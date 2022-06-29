#ifndef IMU_H
#define IMU_H

#include "i2c_imu_device.h"

namespace imu
{
    const double RadToDegree = 57.29577951308;
    const double DegreeToRad = 0.0174533;

    struct Angles
    {
        double pitch = 0;
        double roll = 0;
        double yaw = 0;
    };

    enum class CalibrationType {
        MinMax,
        EllipsoidFit
    };

    class Imu
    {
    public:
        Imu(){};
        virtual bool setup() = 0;
        virtual void start() = 0;
        virtual void finish() = 0;
        virtual void calibrate(CalibrationType type) = 0;

    protected:
        virtual void exec() = 0;

        std::shared_ptr<imu::I2cIMUDevice> magnetometer = nullptr;
        std::shared_ptr<imu::I2cIMUDevice> accelerometer = nullptr;

        Angles angles;
    };
}

#endif // IMU_H