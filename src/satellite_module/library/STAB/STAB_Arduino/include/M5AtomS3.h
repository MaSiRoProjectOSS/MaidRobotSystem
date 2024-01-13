/**
 * @file M5AtomS3.h
 * @brief
 * @version 0.23.12
 * @date 2024-01-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */

#ifndef STAB_M5AtomS3_H
#define STAB_M5AtomS3_H
#include <Arduino.h>

typedef int UBaseType_t;

namespace m5
{
#define BLACK 0x00
#define RED   0xFF

class M5Unified {
public:
    M5Unified();
    struct config_t {};
};
class Display_Class {
public:
    Display_Class()
    {
    }
};
class Lcd_Class {
public:
    Lcd_Class()
    {
    }
    void sleep()
    {
    }
    void wakeup()
    {
    }
    void printf(...)
    {
    }
    void setRotation(int value)
    {
    }
    void clear(uint16_t color = 0)
    {
    }
    void setTextColor(uint16_t color, uint16_t back_color = 0)
    {
    }
    void setCursor(uint16_t x, uint16_t y)
    {
    }
};
class IMU_Class {
public:
    IMU_Class()
    {
    }
    enum sensor_mask_t
    {
        sensor_mask_none  = 0,
        sensor_mask_accel = 1 << 0x01,
        sensor_mask_gyro  = 2 << 0x01,
        sensor_mask_mag   = 3 << 0x01,
    };

    struct imu_3d_t {
        union {
            float value[3];
            struct {
                float x;
                float y;
                float z;
            };
        };
    };
    struct imu_data_t {
        uint32_t usec;
        union {
            float value[9];
            imu_3d_t sensor[3];
            struct {
                imu_3d_t accel;
                imu_3d_t gyro;
                imu_3d_t mag;
            };
        };
    };

public:
    sensor_mask_t update()
    {
        return this->_mask;
    }
    imu_data_t getImuData()
    {
        return this->_data;
    }

public:
    void PUB_update(sensor_mask_t mask)
    {
        this->_mask = mask;
    }
    void PUB_getImuData(imu_data_t data)
    {
        this->_data = data;
    }

private:
    sensor_mask_t _mask = sensor_mask_none;
    imu_data_t _data;
};
class Power_Class {
public:
    Power_Class()
    {
    }
};
class Button_Class {
public:
    Button_Class()
    {
    }
    bool wasPressed()
    {
        if (true == this->_wasPressed) {
            this->_wasPressed = false;
            return true;
        } else {
            return false;
        }
    }
    void PUB_wasPressed()
    {
        this->_wasPressed = true;
    }

private:
    bool _wasPressed = false;
};

class M5AtomS3 {
public:
    M5AtomS3(void)
    {
    }
    ~M5AtomS3(void)
    {
    }

public:
    bool begin(bool ledEnable = false)
    {
        return true;
    }
    bool begin(m5::M5Unified::config_t cfg, bool ledEnable = false)
    {
        return true;
    }
    void update()
    {
    }
    M5Unified::config_t config()
    {
        return M5Unified::config_t();
    };

public:
    Display_Class Display;
    Lcd_Class Lcd;

    IMU_Class Imu;
    Power_Class Power;
    Button_Class BtnA;
};

} // namespace m5

UBaseType_t uxTaskGetStackHighWaterMark(...)
{
    return 0;
}

using namespace m5;

M5AtomS3 M5;
#endif
