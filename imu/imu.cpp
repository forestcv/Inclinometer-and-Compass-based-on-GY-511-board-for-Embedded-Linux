#include "imu.h"
#include "../calibration/min_max_calibration.h"
#include "../calibration/ellipsoid_fit_calibration.h"

#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <functional>
#include <cmath>

using namespace imu;

bool Imu::setup()
{
    accelerometer->sensor->registerCoordinatesHandler(std::bind(&Imu::accCoordinatesHandler,
                                                                this, std::placeholders::_1));
    magnetometer->sensor->registerCoordinatesHandler(std::bind(&Imu::magCoordinatesHandler,
                                                               this, std::placeholders::_1));

    return true;
}

void Imu::start()
{
    // start initialization processes and wait
    accelerometer->init();
    magnetometer->init();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // start accelerometer and magnetometer workflows
    accelerometer->start();
    magnetometer->start();

    // start loop for receiving coordinates and orientation angles
    // calculation
    execution.store(true, std::memory_order_acquire);
    std::thread executionThread(&Imu::exec, this);
    executionThread.detach();
}

void Imu::finish()
{
    // finish accelerometer and magnetometer workflows
    accelerometer->finish();
    magnetometer->finish();
}

void Imu::calibrate(imu::CalibrationType type, int pointsNumber)
{
    calibrationType = type;
    accelerometer->calibrationData =
        std::make_unique<calibration::CalibrationData>(pointsNumber);
    magnetometer->calibrationData =
        std::make_unique<calibration::CalibrationData>(pointsNumber);
}

// loop for receiving coordinates and orientation angles calculation
void Imu::exec()
{
    while (execution.load(std::memory_order_acquire))
    {
        // make calibration or measuremet depended of current mode
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
        // calculate orientation angles
        updateAngles();

        std::cout << angles.pitch << "\t" << angles.roll << "\t" << angles.yaw << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// calculate orientation angles
void Imu::updateAngles()
{
    // correct sensor data with calibration coeficients
    Coordinates<double> correctedAcc =
        calibration::correctCoordinates(accelerometer->coordinates,
                                        accelerometer->offset,
                                        accelerometer->rot);

    Coordinates<double> correctedMag =
        calibration::correctCoordinates(magnetometer->coordinates,
                                        magnetometer->offset,
                                        magnetometer->rot);

    // calculate inclinometer angles
    angles.pitch = std::atan2(static_cast<double>(correctedAcc.x),
                              correctedAcc.z) *
                   imu::RadToDegree;
    angles.roll = std::atan2(static_cast<double>(correctedAcc.y),
                             correctedAcc.z) *
                  imu::RadToDegree;

    // compensate tilt for magnetometer data
    compensateTilt(correctedMag, angles.pitch, angles.roll);

    // calculate yaw
    angles.yaw = std::atan2(static_cast<double>(correctedMag.x), correctedMag.y) *
                 imu::RadToDegree;
}

void Imu::calibration()
{
    // check if all nessessary data is collected
    if (accelerometer->calibrationData->isFull())
    {
        // make calibration of calibrationType 
        switch (calibrationType)
        {
        case CalibrationType::MinMax:
        default:
            accelerometer->offset =
                imu::calibration::min_max_calibration::calibrate(accelerometer->calibrationData);
            break;
        case CalibrationType::EllipsoidFit:
        {
            auto [offset, rot] = imu::calibration::ellipsoid_fit::calibrate(accelerometer->calibrationData);
            accelerometer->offset = offset;
            accelerometer->rot = rot;
        }
        break;
        }
    }
    // add data for calibration
    else
        accelerometer->calibrationData->add(accelerometer->coordinates);

    if (magnetometer->calibrationData->isFull())
    {
        switch (calibrationType)
        {
        case CalibrationType::MinMax:
        default:
            magnetometer->offset =
                imu::calibration::min_max_calibration::calibrate(magnetometer->calibrationData);
            break;
        case CalibrationType::EllipsoidFit:
        {
            auto [offset, rot] = imu::calibration::ellipsoid_fit::calibrate(magnetometer->calibrationData);
            magnetometer->offset = offset;
            magnetometer->rot = rot;
        }
        break;
        }
    }
    else
        magnetometer->calibrationData->add(magnetometer->coordinates);
}

void Imu::accCoordinatesHandler(const imu::Coordinates<int16_t> &coordinates)
{
    accelerometer->coordinates = coordinates;
    // std::cout << "imu:\t" << currentAccCoordinates.x << "\t" << currentAccCoordinates.z << std::endl;
    accelerometer->dataReadyCV.notify_all();
}

void Imu::magCoordinatesHandler(const imu::Coordinates<int16_t> &coordinates)
{
    magnetometer->coordinates = coordinates;
    magnetometer->dataReadyCV.notify_all();
}

void applyOffset(Coordinates<double> &coordinates, const imu::calibration::Offset &offset)
{
    coordinates.x += offset.x;
    coordinates.y += offset.y;
    coordinates.z += offset.z;
}

// https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/trit.2017.0024
void imu::compensateTilt(Coordinates<double> &magCoordinates,
                         double pitch, double roll)
{
    magCoordinates.x = magCoordinates.x * cos(pitch) +
                       magCoordinates.y * sin(pitch) * sin(roll) +
                       magCoordinates.z * sin(pitch) * cos(roll);
    magCoordinates.y = magCoordinates.y * cos(roll) +
                       magCoordinates.z * sin(roll);
}