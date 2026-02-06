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
#include "src/comm/serial_comm.h"

// ======================================================
// OBJECTS
// ======================================================

// Communication
SerialComm serial;

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

// Odometry (observe only)
Odometry odom;

// Velocity command
VelocityCmd velocityCmd(WHEEL_RADIUS_M, TRACK_WIDTH_M);

// Wheel PID
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
    serial.begin(SERIAL_BAUDRATE);
    delay(1500);

    Serial.println(" Hamals Firmware - SERIAL MODE");

    leftEncoder.begin();
    rightEncoder.begin();

    if (!imu.begin()) {
        Serial.println("IMU init failed");
        while (1) {}
    }

    motorL.begin();
    motorR.begin();

    // Wheel PID (baseline)
    pidL.setGains(WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD);
    pidR.setGains(WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD);

    pidL.setOutputLimits(-PWM_MAX, PWM_MAX);
    pidR.setOutputLimits(-PWM_MAX, PWM_MAX);


    pidL.setRampLimit(WHEEL_PID_RAMP_STEP);
    pidR.setRampLimit(WHEEL_PID_RAMP_STEP);

    odom.reset();
    lastLoopMs = millis();

    Serial.println("▶️ Ready for cmd_vel");
}

// ======================================================
// LOOP (ORCHESTRATION ONLY)
// ======================================================
void loop() {
    // --------------------
    // Timing
    // --------------------
    unsigned long now = millis();
    float dt = (now - lastLoopMs) * 0.001f;
    lastLoopMs = now;
    if (dt <= 0.0f) return;

    // --------------------
    // SERIAL RX (ROS → MCU)
    // --------------------
    serial.update();

    float v_target = 0.0f;
    float w_target = 0.0f;

    if (serial.hasCmdVel()) {
        CmdVel cmd = serial.getCmdVel();
        v_target = cmd.v;
        w_target = cmd.w;
    }

    // --------------------
    // IMU
    // --------------------
    imu.update();
    float yaw = imu.getYaw();

    // --------------------
    // YAW CORRECTION (CONFIG-DRIVEN)
    // --------------------
    float w_cmd = w_target;

    if constexpr (ENABLE_YAW_CORRECTION) {
        if (fabs(w_target) < YAW_CORRECTION_W_EPS) {
            float yaw_error = -yaw;
            w_cmd += YAW_CORRECTION_KP * yaw_error;
        }
    }

    // --------------------
    // VELOCITY CMD
    // --------------------
    velocityCmd.setTarget(v_target, w_cmd);
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
    // ODOMETRY
    // --------------------
    odom.update(kinOut.v, yaw, dt);

    // --------------------
    // PID → MOTORS
    // --------------------
    float pwmL = pidL.update(omegaLt, kinOut.omega_left, dt);
    float pwmR = pidR.update(omegaRt, kinOut.omega_right, dt);

    motorL.setPWM((int)pwmL);
    motorR.setPWM((int)pwmR);

    // --------------------
    // SERIAL TX (MCU → ROS)
    // --------------------
    serial.sendOdom(
        odom.x(),
        odom.y(),
        odom.yaw(),
        kinOut.v,
        kinOut.w
    );
}