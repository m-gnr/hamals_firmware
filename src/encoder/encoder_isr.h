#pragma once

#include <Arduino.h>  
#include <stdint.h>

extern volatile int32_t leftPulseCount;
extern volatile int32_t rightPulseCount;

void IRAM_ATTR leftEncoderISR();
void IRAM_ATTR rightEncoderISR();