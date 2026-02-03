#include "wheel_pid.h"
#include "../config/config_robot.h"

WheelPID::WheelPID()
{
    // Defaults are zero; user must set gains and limits
}


void WheelPID::setGains(float kp,
                        float ki,
                        float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void WheelPID::setOutputLimits(float min_pwm,
                               float max_pwm)
{
    output_min_ = min_pwm;
    output_max_ = max_pwm;
}

void WheelPID::reset()
{
    integral_   = 0.0f;
    prev_error_ = 0.0f;
}

float WheelPID::update(float target,
                       float measured,
                       float dt)
{
    if (dt <= 0.0f)
        return 0.0f;

    const float error = target - measured;

    // Integral term
    integral_ += error * dt;

    // Derivative term
    const float derivative = (error - prev_error_) / dt;

    // PID output
    float output =
        (kp_ * error) +
        (ki_ * integral_) +
        (kd_ * derivative);

    // Anti-windup via output clamping
    if (output > output_max_) {
        output = output_max_;
        // Optional: integral rollback could be added later
    }
    else if (output < output_min_) {
        output = output_min_;
    }

    prev_error_ = error;
    return output;
}
