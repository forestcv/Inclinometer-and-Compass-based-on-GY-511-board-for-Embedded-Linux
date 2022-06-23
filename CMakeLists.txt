cmake_minimum_required(VERSION 3.2)

project(gy_511)

# set_target_properties(${PROJECT_NAME}
#                         PROPERTIES
#                         CXX_STANDARD 17
#                         CXX_STANDARD_REQUIRED ON)

add_executable(${PROJECT_NAME}
    main.cpp
    i2c/i2c.c
    i2c/i2c.h
    
    imu/i2c_imu_device.h

    gy_511/accelerometer.cpp
    gy_511/accelerometer.h
    gy_511/magnetometer.cpp
    gy_511/magnetometer.h
    gy_511/registers.h

    )

target_link_libraries(${PROJECT_NAME} -lpthread)