#include "encoder.h"
#include <Arduino.h>

static Encoder* encoderA_instance = nullptr;
static Encoder* encoderB_instance = nullptr;

void IRAM_ATTR encoderA_ISR() {
    if (encoderA_instance)
        encoderA_instance->handleISR();
}

void IRAM_ATTR encoderB_ISR() {
    if (encoderB_instance)
        encoderB_instance->handleISR();
}

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
    : pinA_(pinA), pinB_(pinB) {}

void Encoder::begin() {
    pinMode(pinA_, INPUT_PULLUP);
    pinMode(pinB_, INPUT_PULLUP);

    if (encoderA_instance == nullptr) {
        encoderA_instance = this;
        attachInterrupt(digitalPinToInterrupt(pinA_), encoderA_ISR, CHANGE);
    } else if (encoderB_instance == nullptr) {
        encoderB_instance = this;
        attachInterrupt(digitalPinToInterrupt(pinA_), encoderB_ISR, CHANGE);
    }
}

void Encoder::handleISR() {
    if (digitalRead(pinA_) == digitalRead(pinB_))
        count_++;
    else
        count_--;
}

int32_t Encoder::readDelta() {
    noInterrupts();
    int32_t current = count_;
    int32_t delta   = current - last_count_;
    last_count_     = current;
    interrupts();
    return delta;
}

int32_t Encoder::readTotal() const {
    noInterrupts();
    int32_t total = count_;
    interrupts();
    return total;
}

void Encoder::reset() {
    noInterrupts();
    count_      = 0;
    last_count_ = 0;
    interrupts();
}