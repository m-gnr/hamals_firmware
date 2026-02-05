#pragma once

#include <Adafruit_BNO08x.h>

// =======================================================
// IMU interface (BNO085 - SPI, Polling)
// -------------------------------------------------------
// Responsibility:
//  - Initialize IMU
//  - Provide yaw (rad) and yaw rate (rad/s)
//  - Zero yaw at startup
// =======================================================

class IMU {
public:
    IMU(uint8_t cs_pin,
        uint8_t int_pin,
        uint8_t rst_pin);

    bool begin();

    void update();

    float getYaw() const;
    float getYawRate() const;

private:
    uint8_t cs_;
    uint8_t int_;
    uint8_t rst_;

    Adafruit_BNO08x bno08x_;
    sh2_SensorValue_t sensorValue_;

    float yaw_ = 0.0f;
    float last_yaw_ = 0.0f;
    float yaw_rate_ = 0.0f;

    float yaw_offset_ = 0.0f;
    bool  yaw_zeroed_ = false;

    unsigned long last_time_ms_ = 0;
};