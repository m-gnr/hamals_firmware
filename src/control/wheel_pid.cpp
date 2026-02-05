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
    integral_     = 0;
    prev_error_   = 0;
    last_output_  = 0;
}

float WheelPID::update(float target, float measured, float dt) {
    if (dt <= 0.0f)
        return last_output_;

    float error = target - measured;

    integral_ += error * dt;

    float derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    float output =
        kp_ * error +
        ki_ * integral_ +
        kd_ * derivative;

    if (output > max_out_)
        output = max_out_;
    else if (output < min_out_)
        output = min_out_;


    float delta = output - last_output_;

    if (delta > max_step_)
        output = last_output_ + max_step_;
    else if (delta < -max_step_)
        output = last_output_ - max_step_;

    last_output_ = output;
    return output;
}