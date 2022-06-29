#include "magnetometer.h"

#include <thread>
#include <chrono>

using namespace imu;
using namespace gy511::magnetometer;

Magnetometer::Magnetometer(std::shared_ptr<I2CDevice> device) : I2cIMUDevice(device)
{
    createInitializationQueue();
    initializationQueueIt = initializationQueue.begin();
    coordinatesReadingQueue = createCoordinatesReadingQueue();
    coordinatesReadingQueueIt = coordinatesReadingQueue.begin();
}

void Magnetometer::init()
{
    std::thread initializationThread(&Magnetometer::initialize, this);
    initializationThread.detach();
}

void Magnetometer::start()
{
    execution.store(true, std::memory_order_acquire);
    std::thread executionThread(&Magnetometer::exec, this);
    executionThread.detach();
}

void Magnetometer::finish()
{
    execution.store(false, std::memory_order_acquire);
}

#include <iostream>
void Magnetometer::exec()
{
    while (execution.load(std::memory_order_acquire))
    {
        Coordinates coordinates;
        if (readCoordinates(coordinates))
        {
            // std::cout << coordinates.x << "\t" << coordinates.y << "\t" << coordinates.z << std::endl;
            for (auto handler : handlers)
                handler(coordinates);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
}

void Magnetometer::createInitializationQueue()
{
    initializationQueue.reserve(10);
    initializationQueue.emplace_back(InitializationStages::SET_DO);
    initializationQueue.emplace_back(InitializationStages::SET_GAIN);
    initializationQueue.emplace_back(InitializationStages::SET_MODE);
    initializationQueue.shrink_to_fit();
}

std::vector<CoordinatesReadingStages> Magnetometer::createCoordinatesReadingQueue()
{
    std::vector<CoordinatesReadingStages> queue;
    queue.reserve(6);
    queue.emplace_back(CoordinatesReadingStages::X_L);
    queue.emplace_back(CoordinatesReadingStages::X_H);
    queue.emplace_back(CoordinatesReadingStages::Z_L);
    queue.emplace_back(CoordinatesReadingStages::Z_H);
    queue.emplace_back(CoordinatesReadingStages::Y_L);
    queue.emplace_back(CoordinatesReadingStages::Y_H);

    return queue;
}

void Magnetometer::initialize()
{
    static int attemptionsCounter = 0;
    while (initializationQueueIt != initializationQueue.end())
    {
        bool stageSuccess = false;
        switch (*initializationQueueIt)
        {
        case InitializationStages::SET_DO:
        {
            stageSuccess = writeRegister(Registers::CRA_REG_M, Masks::DO,
                                         static_cast<uint8_t>(DO::HZ_30_00));
            break;
        }
        case InitializationStages::SET_GAIN:
        {
            stageSuccess = writeRegister(Registers::CRB_REG_M, Masks::GN,
                                         static_cast<uint8_t>(GN::GAUSS_4_0));
            break;
        }
        case InitializationStages::SET_MODE:
        {
            stageSuccess = writeRegister(Registers::MR_REG_M, Masks::MD,
                                         static_cast<uint8_t>(MD::Continuous));
            break;
        }
        default:
            break;
        }

        if (stageSuccess)
        {
            initializationQueueIt++;
            attemptionsCounter = 0;
        }
        else
        {
            attemptionsCounter++;
        }
    }
    std::cout << "init ok" << std::endl;
}

bool Magnetometer::readCoordinates(Coordinates &coordinates)
{
    static int attemptionsCounter = 0;
    coordinatesReadingQueueIt = coordinatesReadingQueue.begin();
    while (coordinatesReadingQueueIt != coordinatesReadingQueue.end())
    {
        bool stageSuccess = false;
        switch (*coordinatesReadingQueueIt)
        {
        case CoordinatesReadingStages::X_L:
            stageSuccess = readRegister(Registers::OUT_X_L_M, &rawCoordinates.x.bytes[0]);
            break;
        case CoordinatesReadingStages::X_H:
            stageSuccess = readRegister(Registers::OUT_X_H_M, &rawCoordinates.x.bytes[1]);
            break;
        case CoordinatesReadingStages::Y_L:
            stageSuccess = readRegister(Registers::OUT_Y_L_M, &rawCoordinates.y.bytes[0]);
            break;
        case CoordinatesReadingStages::Y_H:
            stageSuccess = readRegister(Registers::OUT_Y_H_M, &rawCoordinates.y.bytes[1]);
            break;
        case CoordinatesReadingStages::Z_L:
            stageSuccess = readRegister(Registers::OUT_Z_L_M, &rawCoordinates.z.bytes[0]);
            break;
        case CoordinatesReadingStages::Z_H:
            stageSuccess = readRegister(Registers::OUT_Z_H_M, &rawCoordinates.z.bytes[1]);
            break;
        default:
            break;
        }
        if (stageSuccess)
        {
            coordinatesReadingQueueIt++;
            attemptionsCounter = 0;
        }
        else
        {
            attemptionsCounter++;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    coordinates.x = rawCoordinates.x.val;
    coordinates.y = rawCoordinates.y.val;
    coordinates.z = rawCoordinates.z.val;

    return true;
}