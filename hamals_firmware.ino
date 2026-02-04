#include <Arduino.h>
#include "src/encoder/encoder.h"
#include "src/encoder/encoder_isr.h"

// ---------------- PIN TANIMLARI ----------------
#define ENC_L_A   D13
#define ENC_L_B   D14
#define ENC_R_A   D8
#define ENC_R_B   D9

// Encoder nesneleri (ISR sayaçlarını okur)
Encoder leftEncoder(&leftPulseCount);
Encoder rightEncoder(&rightPulseCount);

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n=== ENCODER FREE-ISR TEST ===");

    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);

    // ISR bağlama (SADECE BURADA)
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, CHANGE);

    Serial.println("ISR'ler baglandi.\n");
}

void loop() {
    int32_t dL = leftEncoder.readDelta();
    int32_t dR = rightEncoder.readDelta();

    Serial.print("ΔL=");
    Serial.print(dL);
    Serial.print("  totalL=");
    Serial.print(leftEncoder.readTotal());

    Serial.print(" | ΔR=");
    Serial.print(dR);
    Serial.print("  totalR=");
    Serial.println(rightEncoder.readTotal());

    delay(200);
}