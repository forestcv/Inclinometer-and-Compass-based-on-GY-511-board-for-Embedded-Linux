#include <iostream>
#include <cstring>
#include <string>
#include <memory>
#include <thread>

#include "imu/imu.h"
#include "gy_511/gy511_based_imu.h"
#include "gy_511/accelerometer.h"
#include "gy_511/magnetometer.h"
#include "i2c/i2c.h"

int main()
{
    gy511::Gy511BasedIMU imu{};
    if (imu.setup("../settings/main_settings.json"))
    {
        imu.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000000));
    }

    return 0;
}