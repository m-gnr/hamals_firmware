#include "timing.h"
#include <Arduino.h>

Timing::Timing(float period_s)
    : period_s_(period_s),
      last_ms_(0),
      dt_(0.0f)
{
}

bool Timing::tick()
{
    uint32_t now = millis();

    if (last_ms_ == 0) {
        last_ms_ = now;
        return false;
    }

    uint32_t elapsed_ms = now - last_ms_;
    float elapsed_s = elapsed_ms * 0.001f;

    if (elapsed_s < period_s_)
        return false;

    dt_ = elapsed_s;
    last_ms_ = now;
    return true;
}

float Timing::dt() const
{
    return dt_;
}

void Timing::reset()
{
    last_ms_ = 0;
    dt_ = 0.0f;
}