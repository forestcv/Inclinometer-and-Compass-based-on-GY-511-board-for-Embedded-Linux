#ifndef ACCELEROMETER_REGISTERS_H
#define ACCELEROMETER_REGISTERS_H

enum class CoordinatesReadingStages
{
    X_L,
    X_H,
    Y_L,
    Y_H,
    Z_L,
    Z_H
};

namespace gy511::accelerometer
{
    enum class Registers
    {
        CTRL_REG1_A = 0x20,
        CTRL_REG2_A = 0x21,
        CTRL_REG3_A = 0x22,
        CTRL_REG4_A = 0x23,
        CTRL_REG5_A = 0x24,
        CTRL_REG6_A = 0x25,
        REFERENCE_A = 0x26,
        STATUS_REG_A = 0x27,
        OUT_X_L_A = 0x28,
        OUT_X_H_A = 0x29,
        OUT_Y_L_A = 0x2A,
        OUT_Y_H_A = 0x2B,
        OUT_Z_L_A = 0x2C,
        OUT_Z_H_A = 0x2D,
    };

    enum class Masks
    {
        ODR = 0xF0,
        LPen = 0x08,
        Zen = 0x04,
        Yen = 0x02,
        Xen = 0x01,
        XYZen = 0x07,
        BDU = 0x80,
        FS = 0x30,
        HR = 0x08
    };

    enum class ODR
    {
        POWER_DOWN = 0x00,
        LOW_POWER_1HZ = 0x10,
        LOW_POWER_10HZ = 0x20,
        LOW_POWER_25HZ = 0x30,
        LOW_POWER_50HZ = 0x40,
        LOW_POWER_100HZ = 0x50,
        LOW_POWER_200HZ = 0x60,
        LOW_POWER_400HZ = 0x70,
        LOW_POWER_1620HZ = 0x80,
        LOW_POWER_5378HZ = 0x90
    };

    enum class FS
    {
        G2 = 0x00,
        G4 = 0x10,
        G8 = 0x20,
        G16 = 0x30
    };

    enum class InitializationStages
    {
        SET_ODR,
        SET_XYZ_ENABLE,
        SET_FULL_SCALE,
        SET_HIGH_RESOLUTION_MODE,
        SET_BDU_ENABLE
    };
};


namespace gy511::magnetometer
{
    enum class Registers
    {
        CRA_REG_M = 0x00,
        CRB_REG_M = 0x01,
        MR_REG_M = 0x02,
        OUT_X_H_M = 0x03,
        OUT_X_L_M = 0x04,
        OUT_Z_H_M = 0x05,
        OUT_Z_L_M = 0x06,
        OUT_Y_H_M = 0x07,
        OUT_Y_L_M = 0x08,
        SR_REG_M = 0x09,
        IRA_REG_M = 0X0A,
        IRB_REG_M = 0x0B,
        IRC_REG_M = 0x0C,
        TEMP_OUT_H_M = 0x31,
        TEMP_OUT_L_M = 0x32
    };

    enum class Masks
    {
        TEMP_EN = 0x80,
        DO = 0x1C,
        GN = 0xE0,
        MD = 0x03
    };

    enum class DO
    {
        HZ_0_75 = 0x00,
        HZ_1_50 = 0x04,
        HZ_3_00 = 0x08,
        HZ_7_50 = 0x0C,
        HZ_15_00 = 0x10,
        HZ_30_00 = 0x14,
        HZ_75_00 = 0x18,
        HZ_220_00 = 0x1C
    };

    enum class GN
    {
        GAUSS_1_3 = 0x20,
        GAUSS_1_9 = 0x40,
        GAUSS_2_5 = 0x60,
        GAUSS_4_0 = 0x80,
        GAUSS_4_7 = 0xA0,
        GAUSS_5_6 = 0xC0,
        GAUSS_8_1 = 0xE0
    };

    enum class MD
    {
        Continuous = 0x00,
        Single = 0x01,
        Sleep = 0x11
    };

    enum class InitializationStages
    {
        SET_DO,
        SET_GAIN,
        SET_MODE
    };

};

#endif // ACCELEROMETER_REGISTERS_H