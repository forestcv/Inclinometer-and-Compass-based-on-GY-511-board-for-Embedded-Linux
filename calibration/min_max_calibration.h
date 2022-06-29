#ifndef MIN_MAX_CALIBRATION_H
#define MIN_MAX_CALIBRATION_H

#include "../imu/i2c_imu_device.h"

namespace imu::calibration
{
    using CoordinateVector = std::vector<int16_t>;
    using CoordinateVectors = std::tuple<CoordinateVector, CoordinateVector, CoordinateVector>;

    struct Offset
    {
        double x = 0;
        double y = 0;
        double z = 0;
    };

    class MinMaxCalibration
    {
    public:
        Offset calculate(const std::vector<Coordinates> &coordinates);

    private:
        CoordinateVectors CoordinatesToVectors(const std::vector<Coordinates> &coordinates);
        void sortCoordinateVectors(CoordinateVector &x, CoordinateVector &y, CoordinateVector &z);
        Offset findOffset(CoordinateVector &x, CoordinateVector &y, CoordinateVector &z);
        double findOffsetForSingleAxis(const CoordinateVector &coordinates);
    };
};

#endif // MIN_MAX_CALIBRATION_H