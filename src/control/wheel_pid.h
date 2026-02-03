#pragma once

class WheelPID {
public:
    WheelPID();

    void setGains(float kp,
                  float ki,
                  float kd);

    void setOutputLimits(float min_pwm,
                         float max_pwm);

    void reset();

    // PID update
    //
    // target   : desired wheel angular velocity (rad/s)
    // measured : measured wheel angular velocity (rad/s)
    // dt       : control loop delta time (seconds)
    //
    // returns  : PWM output (signed)
    float update(float target,
                 float measured,
                 float dt);

private:
    // PID gains
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;

    // Output limits
    float output_min_ = 0.0f;
    float output_max_ = 0.0f;

    // Internal PID state
    float integral_   = 0.0f;
    float prev_error_ = 0.0f;
};