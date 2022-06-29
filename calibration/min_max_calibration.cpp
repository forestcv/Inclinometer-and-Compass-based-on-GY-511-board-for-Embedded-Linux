#include "min_max_calibration.h"

#include <algorithm>

using namespace imu::calibration;

Offset MinMaxCalibration::calculate(const std::vector<Coordinates> &coordinates)
{
    auto [x, y, z] = CoordinatesToVectors(coordinates);
    sortCoordinateVectors(x, y, z);
    return findOffset(x, y, z);
}

CoordinateVectors MinMaxCalibration::CoordinatesToVectors(const std::vector<Coordinates> &coordinates)
{
    CoordinateVector x, y, z;
    x.reserve(coordinates.size());
    y.reserve(coordinates.size());
    z.reserve(coordinates.size());

    for (auto coordinate : coordinates)
    {
        x.emplace_back(coordinate.x);
        y.emplace_back(coordinate.y);
        z.emplace_back(coordinate.z);
    }

    return CoordinateVectors{x, y, z};
}

void MinMaxCalibration::sortCoordinateVectors(CoordinateVector &x,
                                              CoordinateVector &y,
                                              CoordinateVector &z)
{
    std::sort(x.begin(), x.end());
    std::sort(y.begin(), y.end());
    std::sort(z.begin(), z.end());
}

Offset MinMaxCalibration::findOffset(CoordinateVector &x, CoordinateVector &y, CoordinateVector &z)
{
    Offset offset;

    offset.x = findOffsetForSingleAxis(x);
    offset.y = findOffsetForSingleAxis(y);
    offset.z = findOffsetForSingleAxis(z);

    return offset;
}

double MinMaxCalibration::findOffsetForSingleAxis(const CoordinateVector &coordinates)
{
    double axisOffset = 0;
    if(coordinates.size() > 1)
    {
        axisOffset = abs(coordinates.at(0) - coordinates.at(coordinates.size() - 1)) / 2;
    }
    return axisOffset;
}
