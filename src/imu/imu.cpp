#include "imu.h"
#include <Arduino.h>
#include <SPI.h>
#include <cmath>

IMU::IMU(uint8_t cs_pin,
         uint8_t int_pin,
         uint8_t rst_pin)
    : cs_(cs_pin),
      int_(int_pin),
      rst_(rst_pin),
      bno08x_(rst_pin)
{
}

bool IMU::begin()
{
    if (!bno08x_.begin_SPI(cs_, int_)) {
        return false;
    }

    // 200 Hz internal update
    bno08x_.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);

    delay(200);

    yaw_zeroed_ = false;
    yaw_ = 0.0f;

    return true;
}

void IMU::update()
{
    if (!bno08x_.getSensorEvent(&sensorValue_))
        return;

    if (sensorValue_.sensorId != SH2_GAME_ROTATION_VECTOR)
        return;

    // Quaternion
    const float qi = sensorValue_.un.gameRotationVector.i;
    const float qj = sensorValue_.un.gameRotationVector.j;
    const float qk = sensorValue_.un.gameRotationVector.k;
    const float qr = sensorValue_.un.gameRotationVector.real;

    // Convert quaternion → yaw (rad)
    const float raw_yaw = atan2(
        2.0f * (qi * qj + qk * qr),
        1.0f - 2.0f * (qj * qj + qk * qk)
    );

    // Zero yaw at startup
    if (!yaw_zeroed_) {
        yaw_offset_ = raw_yaw;
        yaw_ = 0.0f;
        yaw_zeroed_ = true;
        return;
    }

    // Apply offset
    yaw_ = raw_yaw - yaw_offset_;

    // Wrap to [-PI, PI]
    while (yaw_ >  M_PI) yaw_ -= 2.0f * M_PI;
    while (yaw_ < -M_PI) yaw_ += 2.0f * M_PI;
}

float IMU::getYaw() const
{
    return yaw_;
}