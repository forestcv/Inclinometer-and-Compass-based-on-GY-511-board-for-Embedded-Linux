cmake_minimum_required(VERSION 3.2)

project(gy_511)

add_executable(${PROJECT_NAME}
    main.cpp
    i2c/i2c.c
    i2c/i2c.h

    imu/imu.cpp
    imu/imu.h
    imu/i2c_imu_device.h

    gy_511/gy511_based_imu.cpp
    gy_511/gy511_based_imu.h
    gy_511/accelerometer.cpp
    gy_511/accelerometer.h
    gy_511/magnetometer.cpp
    gy_511/magnetometer.h
    gy_511/registers.h

    calibration/calibration.h
    calibration/min_max_calibration.cpp
    calibration/min_max_calibration.h
)

set_target_properties(${PROJECT_NAME}
    PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)

target_link_libraries(${PROJECT_NAME} -lpthread)