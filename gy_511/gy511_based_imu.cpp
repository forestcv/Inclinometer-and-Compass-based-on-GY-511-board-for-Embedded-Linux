#include "gy511_based_imu.h"
#include "../i2c/i2c.h"
#include "../imu/i2c_imu_device.h"
#include "accelerometer.h"
#include "magnetometer.h"

#include <cstring>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using namespace gy511;
using namespace gy511::accelerometer;
using namespace gy511::magnetometer;

bool Gy511BasedIMU::setup(const std::string &pathToSettings)
{
    boost::property_tree::ptree root;
    boost::property_tree::read_json(pathToSettings, root);

    std::string device = root.get<std::string>("i2c_device");

    int bus = 1;
    if ((bus = i2c_open(device.c_str())) == -1)
    {
        throw std::runtime_error("Error: can't open i2c");
        return false;
    }

    // parse settings from json configuration file
    imu::I2cDeviceSettings accSettings = imu::parseDeviceSettings("accelerometer", root);
    imu::I2cDeviceSettings magSettings = imu::parseDeviceSettings("magnetometer", root);

    // create accelerometer and magnetometer instances
    accelerometer->sensor = std::make_shared<Accelerometer>(imu::createIMUDevice(accSettings, bus));
    magnetometer->sensor = std::make_shared<Magnetometer>(imu::createIMUDevice(magSettings, bus));

    Imu::setup();

    return true;
}