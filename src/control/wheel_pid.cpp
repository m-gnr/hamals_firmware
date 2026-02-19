#include "wheel_pid.h"

void WheelPID::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void WheelPID::setOutputLimits(float min, float max) {
    min_out_ = min;
    max_out_ = max;
}

void WheelPID::setRampLimit(float max_step) {
    max_step_ = max_step;
}

void WheelPID::reset() {
    integral_        = 0.0f;
    prev_measured_   = 0.0f;
    last_output_     = 0.0f;
}

float WheelPID::update(float target, float measured, float dt) {
    if (dt <= 0.0f)
        return last_output_;

    float error = target - measured;

    // Derivative on measurement (avoids derivative kick on target change)
    float derivative = -(measured - prev_measured_) / dt;

    // Tentative integral (used only for anti-windup check)
    float new_integral = integral_ + error * dt;

    // Raw output with tentative integral — for saturation detection only
    float raw_output =
        kp_ * error +
        ki_ * new_integral +
        kd_ * derivative;

    // Anti-windup: only integrate if not pushing further into saturation
    bool sat_high = raw_output > max_out_;
    bool sat_low  = raw_output < min_out_;

    if (!((sat_high && error > 0) || (sat_low && error < 0))) {
        integral_ = new_integral;
    }

    // Final output with (possibly held) integral
    float output =
        kp_ * error +
        ki_ * integral_ +
        kd_ * derivative;

    // Clamp
    if (output > max_out_)       output = max_out_;
    else if (output < min_out_)  output = min_out_;

    // Ramp limit (slew rate)
    float delta = output - last_output_;
    if (delta > max_step_)       output = last_output_ + max_step_;
    else if (delta < -max_step_) output = last_output_ - max_step_;

    prev_measured_ = measured;
    last_output_   = output;

    return output;
}