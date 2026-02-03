#include "src/encoder/encoder.h"
#include "src/config/config_pins.h"

Encoder leftEncoder(ENC_L_A, ENC_L_B);
Encoder rightEncoder(ENC_R_A, ENC_R_B);

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== ENCODER RAW TEST START ===");

    leftEncoder.begin();
    rightEncoder.begin();
}

void loop() {
    int32_t dL = leftEncoder.readDelta();
    int32_t dR = rightEncoder.readDelta();

    if (dL != 0 || dR != 0) {
        Serial.print("L: ");
        Serial.print(dL);
        Serial.print(" | R: ");
        Serial.println(dR);
    }

    delay(50);  // 20 Hz yeterli
}