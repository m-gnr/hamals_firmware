#include <Arduino.h>
#include <cmath>

// -------------------- CONFIG ---------------------------
#include "src/config/config_pins.h"
#include "src/config/config_robot.h"

// -------------------- MODULES --------------------------
#include "src/encoder/encoder.h"
#include "src/kinematics/kinematics.h"
#include "src/imu/imu.h"
#include "src/control/velocity_cmd.h"
#include "src/control/wheel_pid.h"
#include "src/motor/motor.h"
#include "src/comm/serial_comm.h"
#include "src/timing/timing.h"

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
Timing encTxTimer(ENC_TX_DT_S);
Timing imuTxTimer(IMU_TX_DT_S);

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

    pidL.setDeadzone(PWM_MIN_START_L, PWM_MIN_RUN_L, DEADZONE_CMD_EPS, DEADZONE_MEAS_EPS);
    pidR.setDeadzone(PWM_MIN_START_R, PWM_MIN_RUN_R, DEADZONE_CMD_EPS, DEADZONE_MEAS_EPS);

    controlTimer.reset();
    encTxTimer.reset();
    imuTxTimer.reset();

    Serial.println("Ready");
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

    const float dt = controlTimer.dt();

    // --------------------
    // SERIAL RX (ROS → MCU) (latched)
    // --------------------
    static float v_target = 0.0f;
    static float w_target = 0.0f;

    if (serial.hasCmdVel()) {
        const CmdVel cmd = serial.getCmdVel();
        v_target = cmd.v;
        w_target = cmd.w;
    }

    // --------------------
    // STOP HANDLING: reset PID integrators to avoid creep after cmd_vel=0
    // --------------------
    if (fabs(v_target) < 0.01f && fabs(w_target) < 0.01f) {
        pidL.reset();
        pidR.reset();
    }

    // --------------------
    // ANGULAR CMD (no MCU-side yaw correction; handled upstream if needed)
    // --------------------
    const float w_cmd = w_target;

    // --------------------
    // VELOCITY CMD
    // --------------------
    // Apply last latched command; TIMEOUT clears targets above.
    velocityCmd.setTarget(v_target, w_cmd);
    velocityCmd.update(dt);

    float omegaLt = 0.0f, omegaRt = 0.0f;
    velocityCmd.getWheelTargets(omegaLt, omegaRt);

    // --------------------
    // ENCODERS
    // --------------------
    const int32_t dL = leftEncoder.readDelta();
    const int32_t dR = rightEncoder.readDelta();

    // --------------------
    // KINEMATICS
    // --------------------
    const KinematicsInput kinIn{ dL, dR, dt };
    const KinematicsOutput kinOut = kinematics.update(kinIn);

    // --------------------
    // PID → MOTORS
    // --------------------
    const float pwmL = pidL.update(omegaLt, kinOut.omega_left, dt);
    const float pwmR = pidR.update(omegaRt, kinOut.omega_right, dt);
    motorL.setPWM((int)pwmL);
    motorR.setPWM((int)pwmR);

    // --------------------
    // SERIAL TX (MCU → ROS)
    // Contract:
    //   $ENC,t_us,dl,dr*CS @ ENC_TX_DT_S
    //   $IMU,t_us,gz,ax,ay,az*CS @ IMU_TX_DT_S
    // --------------------
    bool sent_any = false;
    uint32_t t_us = 0;

    if (encTxTimer.tick()) {
        if (!sent_any) { t_us = micros(); sent_any = true; }
        serial.sendEnc(t_us, dL, dR);
    }

    if (imuTxTimer.tick()) {
        if (!sent_any) { t_us = micros(); sent_any = true; }
        serial.sendImu(t_us, imu.getGz(), imu.getAx(), imu.getAy(), imu.getAz());
    }

}