#pragma once

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// =======================================================
// IMU interface (BNO085 - SPI, Polling)
// -------------------------------------------------------
// Responsibility:
//  - Initialize IMU
//  - Provide gyro z (rad/s)
//  - Provide linear acceleration (m/s^2)
//
// NOTE:
//  - No time handling
//  - All timing handled in control loop
// =======================================================

class IMU {
public:
    IMU(uint8_t cs_pin,
        uint8_t int_pin,
        uint8_t rst_pin);

    bool begin();
    void update();

    // Gyro yaw-rate (rad/s)
    float getGz() const;

    // Linear acceleration (m/s^2) in IMU frame
    float getAx() const;
    float getAy() const;
    float getAz() const;

private:
    uint8_t cs_;
    uint8_t int_;
    uint8_t rst_;

    Adafruit_BNO08x bno08x_;
    sh2_SensorValue_t sensorValue_;

    // Latest IMU measurements (SI units)
    float gz_ = 0.0f;  // rad/s
    float ax_ = 0.0f;  // m/s^2
    float ay_ = 0.0f;  // m/s^2
    float az_ = 0.0f;  // m/s^2
};