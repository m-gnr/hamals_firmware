#include "encoder_isr.h"
#include <Arduino.h>
#include "../config/config_pins.h"

volatile int32_t leftPulseCount  = 0;
volatile int32_t rightPulseCount = 0;

void IRAM_ATTR leftEncoderISR() {
    if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B))
        leftPulseCount--;   
    else
        leftPulseCount++;   
}

void IRAM_ATTR rightEncoderISR() {
    if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B))
        rightPulseCount--;
    else
        rightPulseCount++;
}