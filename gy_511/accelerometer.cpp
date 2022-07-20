#include "accelerometer.h"

#include <thread>
#include <chrono>

using namespace gy511::accelerometer;
using namespace imu;

Accelerometer::Accelerometer(std::shared_ptr<I2CDevice> device) : I2cIMUDevice(device)
{
    // create queue of initialization stages
    createInitializationQueue();
    // set iterator of initialization stages to the beginning of queue
    initializationQueueIt = initializationQueue.begin();
    coordinatesReadingQueue = createCoordinatesReadingQueue();
    coordinatesReadingQueueIt = coordinatesReadingQueue.begin();
}

void Accelerometer::init()
{
    // start loop for initialization workflow
    std::thread initializationThread(&Accelerometer::initialize, this);
    initializationThread.detach();
}

void Accelerometer::start()
{
    // start loop for sensor polling
    execution.store(true, std::memory_order_acquire);
    std::thread executionThread(&Accelerometer::exec, this);
    executionThread.detach();
}

// create queue of initialization stages
void Accelerometer::createInitializationQueue()
{
    initializationQueue.reserve(10);
    initializationQueue.emplace_back(InitializationStages::SET_ODR);
    initializationQueue.emplace_back(InitializationStages::SET_XYZ_ENABLE);
    initializationQueue.emplace_back(InitializationStages::SET_FULL_SCALE);
    initializationQueue.emplace_back(InitializationStages::SET_HIGH_RESOLUTION_MODE);
    initializationQueue.emplace_back(InitializationStages::SET_BDU_ENABLE);
    initializationQueue.shrink_to_fit();
}

// create queue of coordinates reading stages
std::vector<CoordinatesReadingStages> Accelerometer::createCoordinatesReadingQueue()
{
    std::vector<CoordinatesReadingStages> queue;
    queue.reserve(6);
    queue.emplace_back(CoordinatesReadingStages::X_L);
    queue.emplace_back(CoordinatesReadingStages::X_H);
    queue.emplace_back(CoordinatesReadingStages::Y_L);
    queue.emplace_back(CoordinatesReadingStages::Y_H);
    queue.emplace_back(CoordinatesReadingStages::Z_L);
    queue.emplace_back(CoordinatesReadingStages::Z_H);

    return queue;
}

void Accelerometer::initialize()
{
    static int attemptionsCounter = 0;
    while (initializationQueueIt != initializationQueue.end())
    {
        bool stageSuccess = false;
        switch (*initializationQueueIt)
        {
        case InitializationStages::SET_ODR:
        {
            stageSuccess = writeRegister(Registers::CTRL_REG1_A, Masks::ODR,
                                         static_cast<uint8_t>(ODR::LOW_POWER_25HZ));
            break;
        }
        case InitializationStages::SET_XYZ_ENABLE:
        {
            stageSuccess = writeRegister(Registers::CTRL_REG1_A, Masks::XYZen, 0x07);
            break;
        }
        case InitializationStages::SET_FULL_SCALE:
        {
            stageSuccess = writeRegister(Registers::CTRL_REG4_A, Masks::FS,
                                         static_cast<uint8_t>(FS::G2));
            break;
        }
        case InitializationStages::SET_HIGH_RESOLUTION_MODE:
        {
            stageSuccess = writeRegister(Registers::CTRL_REG4_A, Masks::HR, 0x08);
            break;
        }
        case InitializationStages::SET_BDU_ENABLE:
        {
            stageSuccess = writeRegister(Registers::CTRL_REG4_A, Masks::BDU, 0x80);
            break;
        }
        default:
            break;
        }

        if (stageSuccess)
        {
            // go to next stage
            initializationQueueIt++;
            // clear attempts counter
            attemptionsCounter = 0;
        }
        else
        {
            // increment attempts counter
            attemptionsCounter++;
        }
    }
    std::cout << "init ok" << std::endl;
}

bool Accelerometer::readCoordinates(Coordinates<int16_t> &coordinates)
{
    static int attemptionsCounter = 0;
    coordinatesReadingQueueIt = coordinatesReadingQueue.begin();
    while (coordinatesReadingQueueIt != coordinatesReadingQueue.end())
    {
        bool stageSuccess = false;
        switch (*coordinatesReadingQueueIt)
        {
        case CoordinatesReadingStages::X_L:
            stageSuccess = readRegister(Registers::OUT_X_L_A, &rawCoordinates.x.bytes[0]);
            break;
        case CoordinatesReadingStages::X_H:
            stageSuccess = readRegister(Registers::OUT_X_H_A, &rawCoordinates.x.bytes[1]);
            break;
        case CoordinatesReadingStages::Y_L:
            stageSuccess = readRegister(Registers::OUT_Y_L_A, &rawCoordinates.y.bytes[0]);
            break;
        case CoordinatesReadingStages::Y_H:
            stageSuccess = readRegister(Registers::OUT_Y_H_A, &rawCoordinates.y.bytes[1]);
            break;
        case CoordinatesReadingStages::Z_L:
            stageSuccess = readRegister(Registers::OUT_Z_L_A, &rawCoordinates.z.bytes[0]);
            break;
        case CoordinatesReadingStages::Z_H:
            stageSuccess = readRegister(Registers::OUT_Z_H_A, &rawCoordinates.z.bytes[1]);
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

        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

    coordinates.x = rawCoordinates.x.val;
    coordinates.y = rawCoordinates.y.val;
    coordinates.z = rawCoordinates.z.val;

    return true;
}