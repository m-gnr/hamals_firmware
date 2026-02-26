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
#include "src/timing/timing.h"
#include "src/safety/watchdog.h"

// ======================================================
// OBJECTS
// ======================================================

// Communication
SerialComm serial;

// Encoders
Encoder leftEncoder(ENC_L_A, ENC_L_B, ENC_L_DIRECTION);
Encoder rightEncoder(ENC_R_A, ENC_R_B, ENC_R_DIRECTION);  

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

// Wheel PID
WheelPID pidL;
WheelPID pidR;

// Motors
Motor motorL(MOTOR_L_IN1, MOTOR_L_IN2, PWM_MAX);
Motor motorR(MOTOR_R_IN1, MOTOR_R_IN2, PWM_MAX);

// -------------------- TIMING ---------------------------
Timing controlTimer(CONTROL_DT_S);
Timing odomTxTimer(ODOM_TX_DT_S);
Watchdog cmdWatchdog(CMD_VEL_TIMEOUT_S);

// -------------------- CONTROL STATE -------------------
float last_yaw = 0.0f;
float yaw_rate = 0.0f;

// ======================================================
// SETUP
// ======================================================
void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    serial.begin(SERIAL_BAUDRATE);
    delay(1500);

    Serial.println("Hamals Firmware - SERIAL MODE");

    leftEncoder.begin();
    rightEncoder.begin();

    if (!imu.begin()) {
        Serial.println("IMU init failed");
        while (1) {}
    }

    motorL.begin();
    motorR.begin();

    pidL.setGains(WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD);
    pidR.setGains(WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD);

    pidL.setOutputLimits(-PWM_MAX, PWM_MAX);
    pidR.setOutputLimits(-PWM_MAX, PWM_MAX);

    pidL.setRampLimit(WHEEL_PID_RAMP_STEP);
    pidR.setRampLimit(WHEEL_PID_RAMP_STEP);

    odom.reset();
    controlTimer.reset();
    odomTxTimer.reset();
    cmdWatchdog.reset();

    Serial.println("Ready for cmd_vel");
}

// ======================================================
// LOOP (ORCHESTRATION ONLY)
// ======================================================
void loop() {
    // --------------------
    // FAST ASYNC (IO)
    // --------------------
    serial.update();
    imu.update();

    // --------------------
    // FIXED CONTROL LOOP
    // --------------------
    if (!controlTimer.tick())
        return;

    float dt = controlTimer.dt();

    // --------------------
    // SERIAL RX (ROS → MCU)
    // --------------------
    float v_target = 0.0f;
    float w_target = 0.0f;

    if (serial.hasCmdVel()) {
        CmdVel cmd = serial.getCmdVel();
        v_target = cmd.v;
        w_target = cmd.w;
        cmdWatchdog.feed();
    }

    // --------------------
    // WATCHDOG (STATE MACHINE)
    // --------------------
    Watchdog::State wd_state = cmdWatchdog.state(CMD_VEL_GRACE_S);

    if (wd_state == Watchdog::State::TIMEOUT) {
        velocityCmd.reset();
        pidL.reset();
        pidR.reset();
        motorL.stop();
        motorR.stop();
    }

    // --------------------
    // IMU (yaw wrap IMU sınıfında yapılıyor)
    // --------------------
    float yaw = imu.getYaw();

    // --------------------
    // YAW RATE
    // --------------------
    float dyaw = yaw - last_yaw;

    // Wrap-aware difference
    if (dyaw > M_PI)       dyaw -= 2.0f * M_PI;
    else if (dyaw < -M_PI) dyaw += 2.0f * M_PI;

    // --------------------
    // IMU SPIKE GUARD
    // --------------------
    if (fabs(dyaw) > IMU_SPIKE_THRESHOLD_RAD) {
        // Reject spike → freeze yaw change for this cycle
        dyaw = 0.0f;
        yaw = last_yaw;
    }

    if (dt > 0.0f) {
        yaw_rate = dyaw / dt;
    } else {
        yaw_rate = 0.0f;
    }

    last_yaw = yaw;

    // --------------------
    // YAW CORRECTION
    // --------------------
    float w_cmd = w_target;

    if constexpr (ENABLE_YAW_CORRECTION) {
        if (fabs(w_target) < YAW_CORRECTION_W_THRESHOLD) {
            float yaw_error = -yaw;
            w_cmd += YAW_CORRECTION_KP * yaw_error;
        }
    }

    // --------------------
    // VELOCITY CMD
    // --------------------
    if (wd_state == Watchdog::State::GRACE) {
        velocityCmd.setTarget(0.0f, 0.0f);
    }
    else if (wd_state == Watchdog::State::OK) {
        velocityCmd.setTarget(v_target, w_cmd);
    }

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
    if (wd_state != Watchdog::State::TIMEOUT) {
        float pwmL = pidL.update(omegaLt, kinOut.omega_left, dt);
        float pwmR = pidR.update(omegaRt, kinOut.omega_right, dt);

        motorL.setPWM((int)pwmL);
        motorR.setPWM((int)pwmR);
    }

    // --------------------
    // SERIAL TX (MCU → ROS) — 50Hz
    // --------------------
    if (odomTxTimer.tick()) {
        uint32_t t_us = micros();  // <-- gönderme anı timestamp
        serial.sendOdom(
            t_us,
            odom.x(),
            odom.y(),
            odom.yaw(),
            kinOut.v,
            yaw_rate
        );
    }
}