#pragma once

class WheelPID {
public:
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void setRampLimit(float max_step);   // 👈 EKLENDİ
    void reset();

    float update(float target, float measured, float dt);

private:
    float kp_ = 0, ki_ = 0, kd_ = 0;
    float min_out_ = -255, max_out_ = 255;

    float integral_ = 0;
    float prev_error_ = 0;

    // Ramp limiter
    float last_output_ = 0;
    float max_step_ = 20.0f;   // PWM 
};