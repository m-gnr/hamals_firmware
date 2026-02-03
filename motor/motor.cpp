#include "motor.h"
#include <Arduino.h>

Motor::Motor(uint8_t in1,
             uint8_t in2,
             int pwm_max)
    : in1_(in1),
      in2_(in2),
      pwm_max_(pwm_max)
{
}

void Motor::begin()
{
    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);

    // Ensure motor is stopped on startup
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);
}

void Motor::setPWM(int pwm)
{
    // Clamp PWM to allowed range
    if (pwm > pwm_max_)  pwm = pwm_max_;
    if (pwm < -pwm_max_) pwm = -pwm_max_;

    if (pwm > 0) {
        // Forward
        analogWrite(in1_, pwm);
        digitalWrite(in2_, LOW);
    }
    else if (pwm < 0) {
        // Reverse
        analogWrite(in2_, -pwm);
        digitalWrite(in1_, LOW);
    }
    else {
        // Stop
        digitalWrite(in1_, LOW);
        digitalWrite(in2_, LOW);
    }
}

void Motor::stop()
{
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);
}