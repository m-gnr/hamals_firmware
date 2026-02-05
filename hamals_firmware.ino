#include <Arduino.h>

#include "src/config/config_pins.h"
#include "src/config/config_robot.h"

#include "src/encoder/encoder.h"
#include "src/encoder/encoder_isr.h"
#include "src/kinematics/kinematics.h"

// --------------------------------------------------
// Encoder wrappers (ISR sayaçlarını okur)
// --------------------------------------------------
Encoder leftEncoder(&leftPulseCount);
Encoder rightEncoder(&rightPulseCount);

// --------------------------------------------------
// Kinematics object
// --------------------------------------------------
Kinematics kinematics(
    ENCODER_CPR,
    WHEEL_RADIUS_M,
    TRACK_WIDTH_M
);

// --------------------------------------------------
unsigned long lastMillis = 0;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n=== KINEMATICS TEST ===");
    Serial.println("Tekerleri ELLE cevir");
    Serial.println("Ileri / geri / saga / sola dene\n");

    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);

    // 🔴 ISR bağlama SADECE BURADA
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, CHANGE);

    lastMillis = millis();
}

void loop() {
    unsigned long now = millis();
    float dt = (now - lastMillis) / 1000.0f;

    if (dt < CONTROL_DT_S)
        return;

    lastMillis = now;

    // -------------------------------
    // Encoder pulse deltas
    // -------------------------------
    int32_t deltaL = leftEncoder.readDelta();
    int32_t deltaR = rightEncoder.readDelta();

    // -------------------------------
    // Kinematics input
    // -------------------------------
    KinematicsInput kinIn;
    kinIn.delta_left  = deltaL;
    kinIn.delta_right = deltaR;
    kinIn.dt          = dt;

    KinematicsOutput kinOut = kinematics.update(kinIn);

    // -------------------------------
    // DEBUG OUTPUT
    // -------------------------------
    Serial.print("ΔL=");
    Serial.print(deltaL);
    Serial.print(" ΔR=");
    Serial.print(deltaR);

    Serial.print(" | ωL=");
    Serial.print(kinOut.omega_left, 2);
    Serial.print(" rad/s");

    Serial.print(" ωR=");
    Serial.print(kinOut.omega_right, 2);
    Serial.print(" rad/s");

    Serial.print(" | v=");
    Serial.print(kinOut.v, 3);
    Serial.print(" m/s");

    Serial.print(" w=");
    Serial.print(kinOut.w, 3);
    Serial.println(" rad/s");
}