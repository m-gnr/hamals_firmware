#pragma once
#include <stdint.h>

class Encoder {
public:
    explicit Encoder(volatile int32_t* counter);

    int32_t readDelta();
    int32_t readTotal() const;
    void reset();

private:
    volatile int32_t* count_;
    int32_t last_count_ = 0;
};