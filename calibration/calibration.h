#ifndef CALIBRATION_H
#define CALIBRATION_H

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

    struct RotationMatrix
    {
        double m00 = 1;
        double m01 = 0;
        double m02 = 0;

        double m10 = 0;
        double m11 = 1;
        double m12 = 0;

        double m20 = 0;
        double m21 = 0;
        double m22 = 0;
    };

    Coordinates<double> correctCoordinates(Coordinates<int16_t> &coordinates,
                                           const Offset &offset,
                                           const RotationMatrix &rot);

    struct CalibrationData
    {
    public:
        CalibrationData(int size) : size_(size)
        {
            x.reserve(size);
            y.reserve(size);
            z.reserve(size);
        }

        void add(const imu::Coordinates<int16_t> coordinates)
        {
            x.emplace_back(coordinates.x);
            y.emplace_back(coordinates.y);
            z.emplace_back(coordinates.z);
        }

        bool isFull()
        {
            return x.size() >= size_;
        }

        CoordinateVectors get()
        {
            return std::tuple{x, y, z};
        }

        int size()
        {
            return x.size();
        }

    private:
        CoordinateVector x, y, z;
        int size_ = 200;
    };
}

#endif // CALIBRATION_H