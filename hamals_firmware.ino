#include <Arduino.h>

#include "src/config/config_pins.h"
#include "src/config/config_robot.h"

#include "src/encoder/encoder.h"
#include "src/kinematics/kinematics.h"
#include "src/control/velocity_cmd.h"
#include "src/control/wheel_pid.h"
#include "src/motor/motor.h"

// --------------------------------------------------
// TEST AYARLARI
// --------------------------------------------------
constexpr float TARGET_V = 0.40f;   // m/s  (ileri hız)
constexpr float TARGET_W = 0.0f;    // rad/s (düz)
constexpr float DT_S     = CONTROL_DT_S;

// --------------------------------------------------
// ENCODER (PIN TABANLI)
// --------------------------------------------------
Encoder leftEncoder (ENC_L_A, ENC_L_B);
Encoder rightEncoder(ENC_R_A, ENC_R_B);

// --------------------------------------------------
// KINEMATICS
// --------------------------------------------------
Kinematics kinematics(
    ENCODER_CPR_LEFT,
    ENCODER_CPR_RIGHT,
    WHEEL_RADIUS_M,
    TRACK_WIDTH_M
);

// --------------------------------------------------
// VELOCITY COMMAND
// --------------------------------------------------
VelocityCmd velocityCmd(WHEEL_RADIUS_M, TRACK_WIDTH_M);

// --------------------------------------------------
// PID
// --------------------------------------------------
WheelPID pidLeft;
WheelPID pidRight;

// --------------------------------------------------
// MOTOR
// --------------------------------------------------
Motor motorLeft (MOTOR_L_IN1, MOTOR_L_IN2, PWM_MAX);
Motor motorRight(MOTOR_R_IN1, MOTOR_R_IN2, PWM_MAX);

// --------------------------------------------------
unsigned long lastMillis = 0;

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    delay(1500);

    Serial.println();
    Serial.println("====================================");
    Serial.println(" TEST-2 : DUAL WHEEL PID (v sabit)");
    Serial.println("====================================");
    Serial.println("⚠️  Robot HAVADA olmalı");
    Serial.print ("🎯 v = ");
    Serial.print (TARGET_V);
    Serial.print (" m/s , w = ");
    Serial.println(TARGET_W);
    Serial.println();

    // Encoder
    leftEncoder.begin();
    rightEncoder.begin();

    // Motor
    motorLeft.begin();
    motorRight.begin();

    // PID ayarları
    pidLeft.setGains (30.0f, 12.0f, 0.0f);
    pidRight.setGains(30.0f, 12.0f, 0.0f);

    pidLeft.setRampLimit(0.03f);   // 👈 YUMUŞAK
    pidRight.setRampLimit(0.03f);

    pidLeft.setOutputLimits (-PWM_MAX, PWM_MAX);
    pidRight.setOutputLimits(-PWM_MAX, PWM_MAX);

    pidLeft.reset();
    pidRight.reset();

    // Velocity target
    velocityCmd.setTarget(TARGET_V, TARGET_W);

    lastMillis = millis();
}

void loop() {
    unsigned long now = millis();
    float dt = (now - lastMillis) / 1000.0f;

    if (dt < DT_S)
        return;

    lastMillis = now;

    // --------------------------------------------------
    // 1️⃣ Encoder → Kinematics
    // --------------------------------------------------
    int32_t dL = leftEncoder.readDelta();
    int32_t dR = rightEncoder.readDelta();

    KinematicsInput kinIn;
    kinIn.delta_left  = dL;
    kinIn.delta_right = dR;
    kinIn.dt          = dt;

    KinematicsOutput kinOut = kinematics.update(kinIn);

    float omegaL_measured = kinOut.omega_left;
    float omegaR_measured = kinOut.omega_right;

    // --------------------------------------------------
    // 2️⃣ VelocityCmd → wheel targets
    // --------------------------------------------------
    velocityCmd.update(dt);

    float omegaL_target, omegaR_target;
    velocityCmd.getWheelTargets(omegaL_target, omegaR_target);

    // --------------------------------------------------
    // 3️⃣ PID
    // --------------------------------------------------
    float pwmL = pidLeft.update(
        omegaL_target,
        omegaL_measured,
        dt
    );

    float pwmR = pidRight.update(
        omegaR_target,
        omegaR_measured,
        dt
    );

    // --------------------------------------------------
    // 4️⃣ Motor
    // --------------------------------------------------
    motorLeft.setPWM ((int)pwmL);
    motorRight.setPWM((int)pwmR);

    // --------------------------------------------------
    // 5️⃣ DEBUG
    // --------------------------------------------------
    Serial.print("ω_t=");
    Serial.print(omegaL_target, 2);

    Serial.print(" | ωL=");
    Serial.print(omegaL_measured, 2);
    Serial.print(" pwmL=");
    Serial.print(pwmL, 1);

    Serial.print(" || ωR=");
    Serial.print(omegaR_measured, 2);
    Serial.print(" pwmR=");
    Serial.println(pwmR, 1);
}