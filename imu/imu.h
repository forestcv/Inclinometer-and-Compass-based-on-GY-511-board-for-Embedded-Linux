#ifndef IMU_H
#define IMU_H

#include <atomic>
#include <condition_variable>
#include <mutex>

#include "imu_sensor.h"
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

    enum class CalibrationType
    {
        MinMax,
        EllipsoidFit
    };

    enum class Mode
    {
        MEASUREMENT,
        CALIBRATION
    };

    void compensateTilt(Coordinates<double> &magCoordinates,
                                        double pitch, double roll);

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

        void accCoordinatesHandler(const imu::Coordinates<int16_t> &coordinates);
        void magCoordinatesHandler(const imu::Coordinates<int16_t> &coordinates);

        std::atomic<bool> execution;

        std::shared_ptr<imu::ImuSensor> magnetometer = nullptr;
        std::shared_ptr<imu::ImuSensor> accelerometer = nullptr;

        Angles angles;

        // imu::calibration::Offset offset;
        // imu::calibration::Offset rot;
        imu::Mode mode = imu::Mode::MEASUREMENT;
        imu::CalibrationType calibrationType = imu::CalibrationType::MinMax;
    };
}

#endif // IMU_H