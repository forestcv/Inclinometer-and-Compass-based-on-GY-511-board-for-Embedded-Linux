#include "min_max_calibration.h"

#include <algorithm>

using namespace imu::calibration;

static double findOffsetForSingleAxis(const CoordinateVector &coordinates)
{
    double axisOffset = 0;
    if (coordinates.size() > 1)
    {
        axisOffset = abs(coordinates.at(0) - coordinates.at(coordinates.size() - 1)) / 2;
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
    sortCoordinateVectors(x, y, z);
    return findOffset(x, y, z);
}
