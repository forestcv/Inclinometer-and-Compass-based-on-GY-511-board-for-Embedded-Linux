#include "imu.h"
#include "../calibration/min_max_calibration.h"

#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <functional>
#include <cmath>

using namespace imu;

bool Imu::setup()
{
    accelerometer->registerCoordinatesHandler(std::bind(&Imu::accCoordinatesHandler,
                                                        this, std::placeholders::_1));
    magnetometer->registerCoordinatesHandler(std::bind(&Imu::magCoordinatesHandler,
                                                       this, std::placeholders::_1));

    return true;
}

void Imu::start()
{
    accelerometer->init();
    magnetometer->init();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    accelerometer->start();
    magnetometer->start();

    execution.store(true, std::memory_order_acquire);
    std::thread executionThread(&Imu::exec, this);
    executionThread.detach();
}

void Imu::finish()
{
    accelerometer->finish();
    magnetometer->finish();
}

void Imu::calibrate(imu::CalibrationType type, int pointsNumber)
{
    calibrationType = type;
    accCalibrationData = std::make_unique<calibration::CalibrationData>(pointsNumber);
    magCalibrationData = std::make_unique<calibration::CalibrationData>(pointsNumber);
}

void Imu::exec()
{
    while (execution.load(std::memory_order_acquire))
    {
        // std::unique_lock<std::mutex> accLock(accMutex);
        // accDataReadyCV.wait(accLock);
        // std::unique_lock<std::mutex> magLock(accMutex);
        // magDataReadyCV.wait(magLock);
        switch (mode)
        {
        case imu::Mode::MEASUREMENT:
        default:
            updateAngles();
            break;
        case imu::Mode::CALIBRATION:
            calibration();
            break;
        }
        updateAngles();

        std::cout << angles.pitch << "\t" << angles.roll << "\t" << angles.yaw << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Imu::updateAngles()
{
    angles.pitch = std::atan2(static_cast<double>(currentAccCoordinates.x),
                              currentAccCoordinates.z) * imu::RadToDegree;
    angles.roll = std::atan2(static_cast<double>(currentAccCoordinates.y),
                             currentAccCoordinates.z) *imu::RadToDegree;
    angles.yaw = std::atan2(static_cast<double>(currentMagCoordinates.x),
                            currentMagCoordinates.y) * imu::RadToDegree;
}

void Imu::calibration()
{
    accCalibrationData->add(currentAccCoordinates);
    if(accCalibrationData->isFull())
    {
        switch(calibrationType)
        {
            case CalibrationType::MinMax:
            default:
                accOffset = imu::calibration::min_max_calibration::calibrate(accCalibrationData);
                break;
            case CalibrationType::EllipsoidFit:
            break;
        }
    }

    magCalibrationData->add(currentMagCoordinates);
    if(magCalibrationData->isFull())
    {
        switch(calibrationType)
        {
            case CalibrationType::MinMax:
            default:
                magOffset = imu::calibration::min_max_calibration::calibrate(magCalibrationData);
                break;
            case CalibrationType::EllipsoidFit:
            break;
        }
    }
}

void Imu::accCoordinatesHandler(const imu::Coordinates &coordinates)
{
    currentAccCoordinates = coordinates;
    // std::cout << "imu:\t" << currentAccCoordinates.x << "\t" << currentAccCoordinates.z << std::endl;
    accDataReadyCV.notify_all();
}

void Imu::magCoordinatesHandler(const imu::Coordinates &coordinates)
{
    currentMagCoordinates = coordinates;
    magDataReadyCV.notify_all();
}