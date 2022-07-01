#ifndef GY511_BASED_IMU
#define GY511_BASED_IMU

#include "../imu/imu.h"
#include "../imu/i2c_imu_device.h"

namespace gy511
{
    class Gy511BasedIMU : public imu::Imu
    {
    public:
        bool setup() override;
    };
}

#endif // GY511_BASED_IMU
