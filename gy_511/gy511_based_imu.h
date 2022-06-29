#ifndef GY511_BASED_IMU
#define GY511_BASED_IMU

#include <atomic>
#include <condition_variable>
#include <mutex>

#include "../imu/imu.h"
#include "../imu/i2c_imu_device.h"

namespace gy511
{

    class Gy511BasedIMU : public imu::Imu
    {
    public:
        bool setup() override;
        void start() override;
        void finish() override;
        void calibrate(imu::CalibrationType type) override;

    private:
        void exec() override;
        void updateAngles();

        void accCoordinatesHandler(const imu::Coordinates &coordinates);
        void magCoordinatesHandler(const imu::Coordinates &coordinates);

        std::atomic<bool> execution;

        std::condition_variable accDataReadyCV;
        std::condition_variable magDataReadyCV;
        std::mutex accMutex;
        std::mutex magMutex;

        imu::Coordinates currentMagCoordinates;
        imu::Coordinates currentAccCoordinates;
    };

}

#endif // GY511_BASED_IMU
