#include "encoder.h"
#include <Arduino.h>

Encoder::Encoder(volatile int32_t* counter)
    : count_(counter) {}

int32_t Encoder::readDelta() {
    noInterrupts();
    int32_t current = *count_;
    int32_t delta   = current - last_count_;
    last_count_     = current;
    interrupts();
    return delta;
}

int32_t Encoder::readTotal() const {
    noInterrupts();
    int32_t total = *count_;
    interrupts();
    return total;
}

void Encoder::reset() {
    noInterrupts();
    *count_     = 0;
    last_count_ = 0;
    interrupts();
}