#include <iostream>
#include <cstring>
#include <string>
#include <memory>
#include <thread>

#include "gy_511/gy511_based_imu.h"
#include "gy_511/accelerometer.h"
#include "gy_511/magnetometer.h"
#include "i2c/i2c.h"

int main()
{
    /*
    int bus = 1;

    if ((bus = i2c_open("/dev/i2c-1")) == -1)
    {
        std::cout << "I2C open error" << std::endl;
    }

    I2CDevice device;
    auto accPtr = std::make_shared<i2c_device>(device);
    std::memset(&device, 0, sizeof(device));

    accPtr->bus = bus;
    accPtr->addr = 0x19;
    accPtr->iaddr_bytes = 1;
    accPtr->page_bytes = 8;
    
    I2CDevice magDevice;
    auto magPtr = std::make_shared<i2c_device>(magDevice);
    std::memset(&magDevice, 0, sizeof(magDevice));

    magPtr->bus = bus;
    magPtr->addr = 0x1e;
    magPtr->iaddr_bytes = 1;
    magPtr->page_bytes = 8;

    gy511::accelerometer::Accelerometer acc(accPtr);

    acc.init();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    acc.start();

    gy511::magnetometer::Magnetometer mag(magPtr);

    mag.init();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    mag.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(10000000));
    i2c_close(bus);
    */

// /*
    gy511::Gy511BasedIMU imu{};
    imu.setup();
    imu.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000000));
    // */

    return 0;
}