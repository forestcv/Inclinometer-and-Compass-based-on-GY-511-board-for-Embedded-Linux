cmake_minimum_required(VERSION 3.2)

project(GY511)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(${PROJECT_NAME}
    main.cpp
    i2c/i2c.c
    i2c/i2c.h

    imu/imu.cpp
    imu/imu.h
    imu/imu_sensor.h
    imu/i2c_imu_device.cpp
    imu/i2c_imu_device.h

    gy_511/gy511_based_imu.cpp
    gy_511/gy511_based_imu.h
    gy_511/accelerometer.cpp
    gy_511/accelerometer.h
    gy_511/magnetometer.cpp
    gy_511/magnetometer.h
    gy_511/registers.h

    calibration/calibration.cpp
    calibration/calibration.h
    calibration/min_max_calibration.cpp
    calibration/min_max_calibration.h
    calibration/ellipsoid_fit_calibration.cpp
    calibration/ellipsoid_fit_calibration.h
)

set_target_properties(${PROJECT_NAME}
    PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen -lpthread)

option(WITH_GTEST "Whether to build Gtest test" ON)
if(WITH_GTEST)
    include(FetchContent)
    FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(CalibrationTests        
                    tests/tests.cpp
                    calibration/ellipsoid_fit_calibration.cpp
                    calibration/min_max_calibration.cpp
                    )

    set_target_properties(CalibrationTests PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
    )
    target_link_libraries(CalibrationTests gtest_main Eigen3::Eigen -lpthread)
endif()
