#include "i2c_imu_device.h"
#include <chrono>
#include <thread>

using namespace imu;

imu::I2cDeviceSettings imu::parseDeviceSettings(const std::string &device_name,
                                                const boost::property_tree::ptree &settingsRoot)
{
    I2cDeviceSettings settings;

    settings.adress = settingsRoot.get<uint8_t>(device_name + ".i2c_adress");
    settings.internalAdress = settingsRoot.get<uint8_t>(device_name + ".internal_address");
    settings.pageBytes = settingsRoot.get<uint8_t>(device_name + ".page_bytes");

    return settings;
}

std::shared_ptr<i2c_device> imu::createIMUDevice(const I2cDeviceSettings &settings,
                                                 int bus)
{
    I2CDevice device;
    auto ptr = std::make_shared<i2c_device>(device);
    std::memset(&device, 0, sizeof(device));

    ptr->bus = bus;                             /* Bus 0 */
    ptr->addr = settings.adress;                /* Slave address is 0x50, 7-bit */
    ptr->iaddr_bytes = settings.internalAdress; /* Device internal address is 1 byte */
    ptr->page_bytes = settings.pageBytes;       /* Device are capable of 16 bytes per page */

    return ptr;
}

void I2cIMUDevice::finish()
{
    // finish loop for sensor polling
    execution.store(false, std::memory_order_acquire);
}

void I2cIMUDevice::exec()
{
    while (execution.load(std::memory_order_acquire))
    {
        Coordinates<int16_t> coordinates;
        if (readCoordinates(coordinates))
        {
            for (auto handler : handlers)
                handler(coordinates);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
}