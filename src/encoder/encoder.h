#pragma once

#include <stdint.h>

class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB, int direction = 1);

    void begin();

    void handleISR_A();
    void handleISR_B();

    int32_t readDelta();
    int32_t readTotal();

    void reset();

private:
    uint8_t pinA_;
    uint8_t pinB_;
    int direction_ = 1;

    volatile int32_t count_ = 0;
    int32_t last_count_ = 0;
};