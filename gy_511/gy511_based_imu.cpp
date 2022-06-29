#include "gy511_based_imu.h"
#include "../i2c/i2c.h"
#include "accelerometer.h"
#include "magnetometer.h"

#include <iostream>
#include <cstring>
#include <string>
#include <memory>
#include <thread>
#include <functional>
#include <cmath>

using namespace gy511;
using namespace gy511::accelerometer;
using namespace gy511::magnetometer;

bool Gy511BasedIMU::setup()
{
    int bus = 1;

    if ((bus = i2c_open("/dev/i2c-1")) == -1)
    {
        std::cout << "I2C open error" << std::endl;
        return false;
    }

    I2CDevice device;
    auto accPtr = std::make_shared<i2c_device>(device);
    std::memset(&device, 0, sizeof(device));

    accPtr->bus = bus;       /* Bus 0 */
    accPtr->addr = 0x19;     /* Slave address is 0x50, 7-bit */
    accPtr->iaddr_bytes = 1; /* Device internal address is 1 byte */
    accPtr->page_bytes = 8;  /* Device are capable of 16 bytes per page */

    I2CDevice magDevice;
    auto magPtr = std::make_shared<i2c_device>(magDevice);
    std::memset(&magDevice, 0, sizeof(magDevice));

    magPtr->bus = bus;       /* Bus 0 */
    magPtr->addr = 0x1e;     /* Slave address is 0x50, 7-bit */
    magPtr->iaddr_bytes = 1; /* Device internal address is 1 byte */
    magPtr->page_bytes = 8;  /* Device are capable of 16 bytes per page */

    accelerometer = std::make_shared<Accelerometer>(accPtr);
    magnetometer = std::make_shared<Magnetometer>(magPtr);

    accelerometer->registerCoordinatesHandler(std::bind(&Gy511BasedIMU::accCoordinatesHandler,
                                                        this, std::placeholders::_1));
    magnetometer->registerCoordinatesHandler(std::bind(&Gy511BasedIMU::magCoordinatesHandler,
                                                       this, std::placeholders::_1));

    return true;
}

void Gy511BasedIMU::start()
{
    accelerometer->init();
    magnetometer->init();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    accelerometer->start();
    // magnetometer->start();

    execution.store(true, std::memory_order_acquire);
    std::thread executionThread(&Gy511BasedIMU::exec, this);
    executionThread.detach();
}

void Gy511BasedIMU::finish()
{
    accelerometer->finish();
    magnetometer->finish();
}

void Gy511BasedIMU::calibrate(imu::CalibrationType type)
{
}

void Gy511BasedIMU::exec()
{
    while (execution.load(std::memory_order_acquire))
    {
        // std::unique_lock<std::mutex> accLock(accMutex);
        // accDataReadyCV.wait(accLock);
        // std::unique_lock<std::mutex> magLock(accMutex);
        // magDataReadyCV.wait(magLock);
        updateAngles();

        std::cout << angles.pitch << "\t" << angles.roll << "\t" << angles.yaw << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Gy511BasedIMU::updateAngles()
{
    angles.pitch = std::atan2(static_cast<double>(currentAccCoordinates.x),
                              static_cast<double>(currentAccCoordinates.z)) * imu::RadToDegree;
    angles.roll = std::atan2(static_cast<double>(currentAccCoordinates.y), currentAccCoordinates.z) * imu::RadToDegree;
    angles.yaw = std::atan2(static_cast<double>(currentMagCoordinates.x), currentMagCoordinates.y) * imu::RadToDegree;
}

void Gy511BasedIMU::accCoordinatesHandler(const imu::Coordinates &coordinates)
{
    currentAccCoordinates = coordinates;
    // std::cout << "imu:\t" << currentAccCoordinates.x << "\t" << currentAccCoordinates.z << std::endl;
    accDataReadyCV.notify_one();
}

void Gy511BasedIMU::magCoordinatesHandler(const imu::Coordinates &coordinates)
{
    currentMagCoordinates = coordinates;
    magDataReadyCV.notify_one();
}