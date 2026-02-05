#include <Arduino.h>

#include "src/config/config_pins.h"
#include "src/config/config_robot.h"

#include "src/encoder/encoder.h"
#include "src/encoder/encoder_isr.h"
#include "src/kinematics/kinematics.h"

// --------------------------------------------------
// Encoder wrappers (OLD ARCH)
// --------------------------------------------------
Encoder leftEncoder(&leftPulseCount);
Encoder rightEncoder(&rightPulseCount);

// --------------------------------------------------
// Kinematics (LEFT / RIGHT CPR)
// --------------------------------------------------
Kinematics kinematics(
    ENCODER_CPR_LEFT,
    ENCODER_CPR_RIGHT,
    WHEEL_RADIUS_M,
    TRACK_WIDTH_M
);

// --------------------------------------------------
unsigned long lastMillis = 0;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n=== KINEMATICS HAND TEST (OLD ENCODER) ===");

    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);

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

    int32_t dL = leftEncoder.readDelta();
    int32_t dR = rightEncoder.readDelta();

    KinematicsInput kinIn{ dL, dR, dt };
    KinematicsOutput out = kinematics.update(kinIn);

    Serial.print("ΔL=");
    Serial.print(dL);
    Serial.print(" ΔR=");
    Serial.print(dR);

    Serial.print(" | ωL=");
    Serial.print(out.omega_left, 2);
    Serial.print(" ωR=");
    Serial.print(out.omega_right, 2);

    Serial.print(" | v=");
    Serial.print(out.v, 3);
    Serial.print(" w=");
    Serial.println(out.w, 3);
}