#include <iostream>
#include <cstring>
#include <string>
#include <memory>
#include <thread>

#include "i2c/i2c.h"
#include "gy_511/accelerometer.h"
#include "gy_511/magnetometer.h"

int main()
{
    int bus = 1;

    if ((bus = i2c_open("/dev/i2c-1")) == -1)
    {
        std::cout << "I2C open error" << std::endl;
    }

    I2CDevice device;
    auto accPtr = std::make_shared<i2c_device>(device);
    std::memset(&device, 0, sizeof(device));

    accPtr->bus = bus;       /* Bus 0 */
    accPtr->addr = 0x19;     /* Slave address is 0x50, 7-bit */
    accPtr->iaddr_bytes = 1; /* Device internal address is 1 byte */
    accPtr->page_bytes = 8;  /* Device are capable of 16 bytes per page */

    /*
    char buffer[1];
    ssize_t size = sizeof(buffer);
    memset(buffer, 0, sizeof(buffer));

    if ((i2c_read(&device, 0x02, buffer, size)) != size)
    {

        std::cout << "Reading error" << std::endl;
    }

    // uint val =  buffer[0];
    std::cout << (uint)buffer[0] << std::endl;

    */

    I2CDevice magDevice;
    auto magPtr = std::make_shared<i2c_device>(magDevice);
    std::memset(&magDevice, 0, sizeof(magDevice));

    magPtr->bus = bus;       /* Bus 0 */
    magPtr->addr = 0x1e;     /* Slave address is 0x50, 7-bit */
    magPtr->iaddr_bytes = 1; /* Device internal address is 1 byte */
    magPtr->page_bytes = 8;  /* Device are capable of 16 bytes per page */

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

    return 0;
}