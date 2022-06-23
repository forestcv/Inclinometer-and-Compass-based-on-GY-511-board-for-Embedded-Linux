#ifndef I2C_IMU_DEVICE_H
#define I2C_IMU_DEVICE_H

#include <memory>

#include "../i2c/i2c.h"

class I2cIMUDevice
{
public:
    I2cIMUDevice(std::shared_ptr<I2CDevice> device) : device(device) {}
    virtual void init() = 0;
    virtual void start() = 0;

protected:
    struct Coordinates
    {
        int16_t x = 0;
        int16_t y = 0;
        int16_t z = 0;
    };

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

    std::shared_ptr<I2CDevice> device = nullptr;

    template <typename Register>
    bool readRegister(const Register &adress, uint8_t *buffer, uint8_t size = 1)
    {
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

#endif // I2C_IMU_DEVICE_H