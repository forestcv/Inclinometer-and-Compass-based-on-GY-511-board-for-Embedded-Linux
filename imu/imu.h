#ifndef IMU_H
#define IMU_H

#include <atomic>
#include <condition_variable>
#include <mutex>

#include "i2c_imu_device.h"
#include "../calibration/calibration.h"

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

    enum class Mode
    {
        MEASUREMENT,
        CALIBRATION
    };

    class Imu
    {
    public:
        Imu(){};
        virtual bool setup();
        void start();
        void finish();
        void calibrate(imu::CalibrationType type, int pointsNumber);

    protected:
        void exec();
        void updateAngles();
        void calibration();
        void minMaxCalibrationStart();

        void accCoordinatesHandler(const imu::Coordinates &coordinates);
        void magCoordinatesHandler(const imu::Coordinates &coordinates);

        std::atomic<bool> execution;

        std::condition_variable accDataReadyCV;
        std::condition_variable magDataReadyCV;
        std::mutex accMutex;
        std::mutex magMutex;

        imu::Coordinates currentMagCoordinates;
        imu::Coordinates currentAccCoordinates;

        std::shared_ptr<imu::I2cIMUDevice> magnetometer = nullptr;
        std::shared_ptr<imu::I2cIMUDevice> accelerometer = nullptr;

        Angles angles;

        imu::calibration::Offset offset;
        imu::Mode mode = imu::Mode::MEASUREMENT;
        imu::CalibrationType calibrationType = imu::CalibrationType::MinMax;

        std::unique_ptr<imu::calibration::CalibrationData> accCalibrationData = nullptr;
        std::unique_ptr<imu::calibration::CalibrationData> magCalibrationData = nullptr;

        imu::calibration::Offset accOffset;
        imu::calibration::Offset magOffset;

        imu::calibration::RotationMatrix accRotation;
        imu::calibration::RotationMatrix magRotation;
    };
}

#endif // IMU_H