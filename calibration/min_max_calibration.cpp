#include "min_max_calibration.h"

#include <algorithm>

using namespace imu::calibration;

static double findOffsetForSingleAxis(const CoordinateVector &coordinates)
{
    double axisOffset = 0;
    if (coordinates.size() > 1)
    {
        auto maxIt = std::max_element(coordinates.begin(),
                                      coordinates.end());
        auto minIt = std::min_element(coordinates.begin(),
                                      coordinates.end());
        axisOffset = (static_cast<double>(*maxIt) + *minIt) / 2;
    }
    return axisOffset;
}

static Offset findOffset(CoordinateVector &x, CoordinateVector &y, CoordinateVector &z)
{
    Offset offset;

    offset.x = findOffsetForSingleAxis(x);
    offset.y = findOffsetForSingleAxis(y);
    offset.z = findOffsetForSingleAxis(z);

    return offset;
}

static void sortCoordinateVectors(CoordinateVector &x,
                                  CoordinateVector &y,
                                  CoordinateVector &z)
{
    std::sort(x.begin(), x.end());
    std::sort(y.begin(), y.end());
    std::sort(z.begin(), z.end());
}

Offset min_max_calibration::calibrate(std::unique_ptr<imu::calibration::CalibrationData> &data)
{
    auto [x, y, z] = data->get();

    auto isOutrageous = [](int16_t val)
    {
        if (abs(val) > 32765)
            return true;
        return false;
    };

    // erase elements that are close to int16 limit
    x.erase(std::remove_if(x.begin(), x.end(), isOutrageous),
            x.end());
    y.erase(std::remove_if(y.begin(), y.end(), isOutrageous),
            y.end());
    z.erase(std::remove_if(z.begin(), z.end(), isOutrageous),
            z.end());

    return findOffset(x, y, z);
}
