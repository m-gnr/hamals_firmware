#include "encoder_isr.h"

volatile int32_t leftPulseCount  = 0;
volatile int32_t rightPulseCount = 0;

void IRAM_ATTR leftEncoderISR() {
    leftPulseCount++;   
}

void IRAM_ATTR rightEncoderISR() {
    rightPulseCount++; 
}