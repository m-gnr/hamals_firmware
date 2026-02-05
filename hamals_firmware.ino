#include <Arduino.h>

#include "src/config/config_pins.h"
#include "src/encoder/encoder.h"

// --------------------------------------------------
// Encoder instances
// --------------------------------------------------
Encoder leftEncoder(ENC_L_A, ENC_L_B);
Encoder rightEncoder(ENC_R_A, ENC_R_B);

// --------------------------------------------------
unsigned long lastPrint = 0;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n=== ENCODER RAW TEST ===");
    Serial.println("Tekerleri ELLE çevir:");
    Serial.println(" - Sol ileri / geri");
    Serial.println(" - Sağ ileri / geri");
    Serial.println(" - 1 tam tur ≈ CPR\n");

    leftEncoder.begin();
    rightEncoder.begin();

    lastPrint = millis();
}

void loop() {
    if (millis() - lastPrint < 200)
        return;

    lastPrint = millis();

    int32_t dL = leftEncoder.readDelta();
    int32_t dR = rightEncoder.readDelta();

    int32_t tL = leftEncoder.readTotal();
    int32_t tR = rightEncoder.readTotal();

    Serial.print("ΔL=");
    Serial.print(dL);
    Serial.print("  TL=");
    Serial.print(tL);

    Serial.print(" | ΔR=");
    Serial.print(dR);
    Serial.print("  TR=");
    Serial.println(tR);
}