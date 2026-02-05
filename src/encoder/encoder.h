#pragma once

#include <stdint.h>

class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB);

    void begin();

    void handleISR();

    int32_t readDelta();

    int32_t readTotal() const;

    void reset();

private:
    uint8_t pinA_;
    uint8_t pinB_;

    volatile int32_t count_ = 0;
    int32_t last_count_ = 0;
};