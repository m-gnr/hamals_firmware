#include "encoder.h"
#include <Arduino.h>

static Encoder* encoder_instance_1 = nullptr;
static Encoder* encoder_instance_2 = nullptr;

void IRAM_ATTR encoderISR_1() {
    if (encoder_instance_1)
        encoder_instance_1->handleISR();
}

void IRAM_ATTR encoderISR_2() {
    if (encoder_instance_2)
        encoder_instance_2->handleISR();
}

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
    : pinA_(pinA), pinB_(pinB) {}

void Encoder::begin() {
    pinMode(pinA_, INPUT_PULLUP);
    pinMode(pinB_, INPUT_PULLUP);

    if (encoder_instance_1 == nullptr) {
        encoder_instance_1 = this;
        attachInterrupt(digitalPinToInterrupt(pinA_), encoderISR_1, CHANGE);
    }
    else if (encoder_instance_2 == nullptr) {
        encoder_instance_2 = this;
        attachInterrupt(digitalPinToInterrupt(pinA_), encoderISR_2, CHANGE);
    }
    else {
        Serial.println("ERROR: Too many Encoder instances!");
    }
}

void Encoder::handleISR() {
    if (digitalRead(pinA_) == digitalRead(pinB_))
        count_--;
    else
        count_++;
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