#include <Arduino.h>

// -------------------- CONFIG ---------------------------
#include "src/config/config_pins.h"
#include "src/config/config_robot.h"

// -------------------- MODULES --------------------------
#include "src/encoder/encoder.h"
#include "src/kinematics/kinematics.h"
#include "src/imu/imu.h"
#include "src/odometry/odometry.h"
#include "src/control/velocity_cmd.h"
#include "src/control/wheel_pid.h"
#include "src/motor/motor.h"

// ======================================================
// OBJECTS
// ======================================================

// Encoders
Encoder leftEncoder(ENC_L_A, ENC_L_B);
Encoder rightEncoder(ENC_R_A, ENC_R_B);

// IMU
IMU imu(IMU_CS, IMU_INT, IMU_RST);

// Kinematics
Kinematics kinematics(
    ENCODER_CPR_LEFT,
    ENCODER_CPR_RIGHT,
    WHEEL_RADIUS_M,
    TRACK_WIDTH_M
);

// Odometry
Odometry odom;

// Velocity command
VelocityCmd velocityCmd(WHEEL_RADIUS_M, TRACK_WIDTH_M);

// PID
WheelPID pidL;
WheelPID pidR;

// Motors
Motor motorL(MOTOR_L_IN1, MOTOR_L_IN2, PWM_MAX);
Motor motorR(MOTOR_R_IN1, MOTOR_R_IN2, PWM_MAX);

// -------------------- TIMING ---------------------------
unsigned long lastLoopMs = 0;

// ======================================================
// SETUP
// ======================================================
void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    delay(1500);

    Serial.println("====================================");
    Serial.println(" TEST-3 : STRAIGHT LINE (v != 0)");
    Serial.println("====================================");

    leftEncoder.begin();
    rightEncoder.begin();

    if (!imu.begin()) {
        Serial.println("IMU init failed");
        while (1) {}
    }

    motorL.begin();
    motorR.begin();

    // PID
    pidL.setGains(18.0f, 5.0f, 0.0f);
    pidR.setGains(18.0f, 5.0f, 0.0f);

    pidL.setOutputLimits(-PWM_MAX, PWM_MAX);
    pidR.setOutputLimits(-PWM_MAX, PWM_MAX);

    pidL.setRampLimit(10.0f);
    pidR.setRampLimit(10.0f);

    odom.reset();
    lastLoopMs = millis();

    Serial.println("test begin");
}

// ======================================================
// LOOP
// ======================================================
void loop() {
    unsigned long now = millis();
    float dt = (now - lastLoopMs) * 0.001f;
    lastLoopMs = now;

    if (dt <= 0.0f)
        return;

    // --------------------
    // IMU
    // --------------------
    imu.update();
    float yaw = imu.getYaw();   // absolute yaw (0 at startup)

    // --------------------
    // TARGET (CONFIG-DRIVEN)
    // --------------------
    float v_target = 0.20f;
    float w_target = 0.0f;
    float w_cmd    = 0.0f;

    if constexpr (ENABLE_YAW_CORRECTION) {
        if (fabs(w_target) < YAW_CORRECTION_W_EPS) {
            float yaw_error = -yaw;   
            w_cmd = YAW_CORRECTION_KP * yaw_error;
        }
    }

    velocityCmd.setTarget(v_target, w_target + w_cmd);
    velocityCmd.update(dt);

    float omegaLt, omegaRt;
    velocityCmd.getWheelTargets(omegaLt, omegaRt);

    // --------------------
    // ENCODERS
    // --------------------
    int32_t dL = leftEncoder.readDelta();
    int32_t dR = rightEncoder.readDelta();

    // --------------------
    // KINEMATICS
    // --------------------
    KinematicsInput kinIn{ dL, dR, dt };
    KinematicsOutput kinOut = kinematics.update(kinIn);

    // --------------------
    // ODOMETRY (OBSERVE)
    // --------------------
    odom.update(kinOut.v, yaw, dt);

    // --------------------
    // PID → MOTOR
    // --------------------
    float pwmL = pidL.update(omegaLt, kinOut.omega_left, dt);
    float pwmR = pidR.update(omegaRt, kinOut.omega_right, dt);

    motorL.setPWM((int)pwmL);
    motorR.setPWM((int)pwmR);

    // --------------------
    // DEBUG
    // --------------------
    Serial.print("ωLt=");
    Serial.print(omegaLt, 2);
    Serial.print(" ωLm=");
    Serial.print(kinOut.omega_left, 2);
    Serial.print(" pwmL=");
    Serial.print((int)pwmL);

    Serial.print(" | ωRt=");
    Serial.print(omegaRt, 2);
    Serial.print(" ωRm=");
    Serial.print(kinOut.omega_right, 2);
    Serial.print(" pwmR=");
    Serial.print((int)pwmR);

    Serial.print(" | yaw=");
    Serial.print(yaw, 3);
    Serial.print(" w_corr=");
    Serial.print(w_cmd, 3);

    Serial.print(" | x=");
    Serial.print(odom.x(), 3);
    Serial.print(" y=");
    Serial.println(odom.y(), 3);

    delay(10); // ~100 Hz
}