#include <Arduino.h>

// -------------------- CONFIG ---------------------------
#include "src/config/config_pins.h"
#include "src/config/config_robot.h"

// -------------------- MODULES --------------------------
#include "src/encoder/encoder.h"
#include "src/kinematics/kinematics.h"
#include "src/imu/imu.h"
#include "src/odometry/odometry.h"

// ======================================================
// OBJECTS
// ======================================================

// Encoders
Encoder leftEncoder(ENC_L_A, ENC_L_B);
Encoder rightEncoder(ENC_R_A, ENC_R_B);

// IMU (SPI, yaw only, zeroed at startup)
IMU imu(IMU_CS, IMU_INT, IMU_RST);

// Kinematics (LEFT & RIGHT CPR SEPARATE)
Kinematics kinematics(
    ENCODER_CPR_LEFT,
    ENCODER_CPR_RIGHT,
    WHEEL_RADIUS_M,
    TRACK_WIDTH_M
);

// Odometry (IMU-assisted 2D)
Odometry odom;

// -------------------- TIMING ---------------------------
unsigned long lastLoopMs = 0;

// ======================================================
// SETUP
// ======================================================
void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    delay(1500);

    Serial.println("====================================");
    Serial.println(" Hamals Firmware - Odometry Test");
    Serial.println("====================================");

    // Encoders
    leftEncoder.begin();
    rightEncoder.begin();

    // IMU
    if (!imu.begin()) {
        Serial.println("IMU init failed");
        while (1) { delay(10); }
    }

    Serial.println("IMU ready (yaw zeroed)");

    // Odometry reset
    odom.reset();

    lastLoopMs = millis();
}

// ======================================================
// LOOP
// ======================================================
void loop() {
    // --------------------
    // DT
    // --------------------
    unsigned long now = millis();
    float dt = (now - lastLoopMs) * 0.001f;
    lastLoopMs = now;

    if (dt <= 0.0f)
        return;

    // --------------------
    // IMU
    // --------------------
    imu.update();
    float yaw = imu.getYaw();   // rad (absolute, 0 at startup)

    // --------------------
    // ENCODER DELTAS
    // --------------------
    int32_t deltaL = leftEncoder.readDelta();
    int32_t deltaR = rightEncoder.readDelta();

    // --------------------
    // KINEMATICS
    // --------------------
    KinematicsInput kin_in;
    kin_in.delta_left  = deltaL;
    kin_in.delta_right = deltaR;
    kin_in.dt          = dt;

    KinematicsOutput kin_out = kinematics.update(kin_in);

    // --------------------
    // ODOMETRY
    // --------------------
    odom.update(
        kin_out.v,   // linear velocity (m/s)
        yaw,         // absolute yaw (rad)
        dt
    );

    // --------------------
    // DEBUG OUTPUT
    // --------------------
    Serial.print("x=");
    Serial.print(odom.x(), 3);
    Serial.print("  y=");
    Serial.print(odom.y(), 3);
    Serial.print("  yaw=");
    Serial.print(odom.yaw(), 3);

    Serial.print(" | v=");
    Serial.print(kin_out.v, 3);
    Serial.print("  w=");
    Serial.print(kin_out.w, 3);

    Serial.print(" | dL=");
    Serial.print(deltaL);
    Serial.print(" dR=");
    Serial.println(deltaR);

    delay(10); // ~100 Hz debug
}