#include "gy511_based_imu.h"
#include "../i2c/i2c.h"
#include "accelerometer.h"
#include "magnetometer.h"

#include <cstring>

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

    Imu::setup();

    return true;
}