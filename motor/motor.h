#pragma once

#include <stdint.h>

class Motor {
public:
    // in1, in2 : H-bridge control pins
    // pwm_max  : maximum absolute PWM value
    Motor(uint8_t in1,
          uint8_t in2,
          int pwm_max);

    void begin();

    //  +pwm -> forward
    //  -pwm -> reverse
    //   0   -> stop
    void setPWM(int pwm);

    // Immediately stop motor
    void stop();

private:
    uint8_t in1_;
    uint8_t in2_;
    int pwm_max_;
};