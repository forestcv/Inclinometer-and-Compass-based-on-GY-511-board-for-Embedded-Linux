#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <memory>
#include <vector>
#include <atomic>

#include "../imu/i2c_imu_device.h"
#include "registers.h"

namespace gy511::accelerometer
{
    class Accelerometer : public imu::I2cIMUDevice
    {
    public:
        Accelerometer(std::shared_ptr<I2CDevice> device);
        void init() override;
        void start() override;
        void finish() override;

    private:
        bool isInited = false;
        std::vector<InitializationStages> initializationQueue;
        std::vector<InitializationStages>::iterator initializationQueueIt;
        std::vector<CoordinatesReadingStages> coordinatesReadingQueue;
        std::vector<CoordinatesReadingStages>::iterator coordinatesReadingQueueIt;

        std::atomic<bool> execution;

        void exec();
        void createInitializationQueue();
        std::vector<CoordinatesReadingStages> createCoordinatesReadingQueue();
        void initialize();
        bool readCoordinates(imu::Coordinates &coordinates);
    };
};

#endif // ACCELEROMETER_H