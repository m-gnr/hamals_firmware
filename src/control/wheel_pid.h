#pragma once

class WheelPID {
public:
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void setRampLimit(float max_step);
    void reset();
    float update(float target, float measured, float dt);

private:
    float kp_           = 0.0f;
    float ki_           = 0.0f;
    float kd_           = 0.0f;
    
    // safe default
    float min_out_      = -255.0f;
    float max_out_      =  255.0f;
    float max_step_     =  10.0f;   

    float integral_     = 0.0f;
    float prev_measured_= 0.0f;     // derivative kick fix
    float last_output_  = 0.0f;
};