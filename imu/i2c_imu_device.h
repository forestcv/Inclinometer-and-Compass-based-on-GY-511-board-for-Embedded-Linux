#ifndef I2C_IMU_DEVICE_H
#define I2C_IMU_DEVICE_H

#include <memory>
#include <functional>
#include <vector>
#include <iostream>
#include <atomic>

#include <boost/property_tree/ptree.hpp>

#include "../i2c/i2c.h"

namespace imu
{
    template <typename T>
    struct Coordinates
    {
        T x = 0;
        T y = 0;
        T z = 0;
    };

    struct I2cDeviceSettings
    {
        uint8_t adress = 0;
        uint8_t internalAdress = 0;
        uint8_t pageBytes = 0;
    };

    I2cDeviceSettings parseDeviceSettings(const std::string &device_name,
                                          const boost::property_tree::ptree &settingsRoot);

    std::shared_ptr<i2c_device> createIMUDevice(const I2cDeviceSettings &settings,
                                                int bus);

    class I2cIMUDevice
    {
    public:
        I2cIMUDevice(std::shared_ptr<I2CDevice> device) : device(device) {}
        virtual void init() = 0;
        virtual void start() = 0;
        void finish();
        void registerCoordinatesHandler(std::function<void(const Coordinates<int16_t> &)> handler)
        {
            handlers.emplace_back(handler);
        }

    protected:
        union RawCoordinate
        {
            uint8_t bytes[2] = {0, 0};
            int16_t val;
        };

        struct RawCoordinates
        {
            RawCoordinate x;
            RawCoordinate y;
            RawCoordinate z;
        } rawCoordinates;

        std::atomic<bool> execution;
        std::shared_ptr<I2CDevice> device = nullptr;
        std::vector<std::function<void(const Coordinates<int16_t> &)>> handlers;

        void exec();

        virtual bool readCoordinates(imu::Coordinates<int16_t> &coordinates) = 0;

        template <typename Register>
        bool readRegister(const Register &adress, uint8_t *buffer, uint8_t size = 1)
        {
            *buffer = 0x00;
            if (i2c_read(device.get(), static_cast<uint8_t>(adress), buffer, size) == size)
            {
                return true;
            }
            return false;
        }

        template <typename Register, typename Mask>
        bool writeRegister(const Register &adress, const Mask &mask, uint8_t data)
        {
            uint8_t buffer = 0x00;
            if (readRegister(adress, &buffer))
            {
                buffer = data | (buffer & (~static_cast<uint8_t>(mask)));
                if ((i2c_write(device.get(), static_cast<uint8_t>(adress), &buffer, 1)) == 1)
                {
                    return true;
                }
            }
            return false;
        }
    };
};

#endif // I2C_IMU_DEVICE_H