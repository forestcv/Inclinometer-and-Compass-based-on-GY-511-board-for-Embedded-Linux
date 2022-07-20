#include <gtest/gtest.h>

#include <memory>
#include <vector>
#include <fstream>
#include <string>
#include <cmath>

#include "../calibration/ellipsoid_fit_calibration.h"
#include "../calibration/min_max_calibration.h"

std::vector<std::string> split(const std::string &str, char d)
{
    std::vector<std::string> r;

    std::string::size_type start = 0;
    std::string::size_type stop = str.find_first_of(d);
    while (stop != std::string::npos)
    {
        r.push_back(str.substr(start, stop - start));

        start = stop + 1;
        stop = str.find_first_of(d, start);
    }

    r.push_back(str.substr(start));

    return r;
}

void convert_test_file()
{
    std::fstream file("../tests/data/mag.txt");
    if (file.is_open())
    {
        std::string line;
        std::vector<double> x, y, z;
        x.reserve(10000);
        y.reserve(10000);
        z.reserve(10000);
        while (std::getline(file, line))
        {
            std::vector<std::string> linedata = split(line, '\t');
            x.push_back(stod(linedata.at(0)));
            y.push_back(stod(linedata.at(1)));
            z.push_back(stod(linedata.at(2)));
        }
        file.close();

        std::ofstream outFile("../tests/data/mag_16.txt");
        if (outFile.is_open())
        {
            for (int i = 0; i < x.size(); i++)
            {
                outFile << (int16_t)(x.at(i) * 10e3) << "\t" 
                        << (int16_t)(y.at(i) * 10e3) << "\t" 
                        << (int16_t)(z.at(i) * 10e3) << "\n";
            }
        }
        outFile.close();
    }
}

TEST(test_calibration, test_min_max_calibration)
{
    std::fstream file("../tests/data/mag_16.txt");
    if (file.is_open())
    {
        std::unique_ptr<imu::calibration::CalibrationData> data =
            std::make_unique<imu::calibration::CalibrationData>(10000);

        std::string line;
        imu::Coordinates<int16_t> coordinates;
        while (std::getline(file, line))
        {
            std::vector<std::string> linedata = split(line, '\t');
            coordinates.x = stoi(linedata.at(0));
            coordinates.y = stoi(linedata.at(1));
            coordinates.z = stoi(linedata.at(2));
            data->add(coordinates);
        }
        file.close();

        auto offset = imu::calibration::min_max_calibration::calibrate(data);

        imu::calibration::Offset expectedOffset;
        expectedOffset.x = -300;
        expectedOffset.y = -310;
        expectedOffset.z = -200;

        EXPECT_NEAR(offset.x, expectedOffset.x, 10e2);
        EXPECT_NEAR(offset.y, expectedOffset.y, 10e2);
        EXPECT_NEAR(offset.z, expectedOffset.z, 10e2);
    }
    else
        FAIL();
}

TEST(test_calibration, test_ellipsoid_fit_calibration)
{
    std::fstream file("../tests/data/mag_16.txt");
    if (file.is_open())
    {
        std::unique_ptr<imu::calibration::CalibrationData> data =
            std::make_unique<imu::calibration::CalibrationData>(10000);

        std::string line;
        imu::Coordinates<int16_t> coordinates;
        while (std::getline(file, line))
        {
            std::vector<std::string> linedata = split(line, '\t');
            coordinates.x = stoi(linedata.at(0));
            coordinates.y = stoi(linedata.at(1));
            coordinates.z = stoi(linedata.at(2));
            data->add(coordinates);
        }
        file.close();

        auto [offset, rot] = imu::calibration::ellipsoid_fit::calibrate(data);

        imu::calibration::Offset expectedOffset;
        expectedOffset.x = -216.617029;
        expectedOffset.y = 132.456709;
        expectedOffset.z = -261.660595;

        imu::calibration::RotationMatrix expectedRot;
        expectedRot.m00 = 1.682055;
        expectedRot.m01 = -0.031304;
        expectedRot.m02 = 0.011530;

        expectedRot.m10 = -0.031304;
        expectedRot.m11 = 1.695332;
        expectedRot.m12 = 0.005820;

        expectedRot.m20 = 0.011530;
        expectedRot.m21 = 0.005820;
        expectedRot.m22 = 1.821197;

        EXPECT_NEAR(offset.x, expectedOffset.x, 10e-1);
        EXPECT_NEAR(offset.y, expectedOffset.y, 10e-1);
        EXPECT_NEAR(offset.z, expectedOffset.z, 10e-1);

        EXPECT_NEAR(rot.m00, expectedRot.m00, 10e-3);
        EXPECT_NEAR(rot.m01, expectedRot.m01, 10e-3);
        EXPECT_NEAR(rot.m02, expectedRot.m02, 10e-3);

        EXPECT_NEAR(rot.m10, expectedRot.m10, 10e-3);
        EXPECT_NEAR(rot.m11, expectedRot.m11, 10e-3);
        EXPECT_NEAR(rot.m12, expectedRot.m12, 10e-3);

        EXPECT_NEAR(rot.m20, expectedRot.m20, 10e-3);
        EXPECT_NEAR(rot.m21, expectedRot.m21, 10e-3);
        EXPECT_NEAR(rot.m22, expectedRot.m22, 10e-3);
    }
    else
        FAIL();
}

int main(int argc, char *argv[])
{
    convert_test_file();
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}