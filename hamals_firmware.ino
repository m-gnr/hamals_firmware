#include <Arduino.h>

// ---------------- CONFIG ----------------
#include "src/config/config_pins.h"
#include "src/config/config_robot.h"

// ---------------- LOW LEVEL ----------------
#include "src/encoder/encoder.h"
#include "src/motor/motor.h"

// ---------------- MID LEVEL ----------------
#include "src/kinematics/kinematics.h"
#include "src/control/wheel_pid.h"
#include "src/control/velocity_cmd.h"

// ---------------- SENSORS ----------------
#include "src/imu/imu.h"


// Encoders
Encoder leftEncoder(ENC_L_A, ENC_L_B);
Encoder rightEncoder(ENC_R_A, ENC_R_B);

// Motors
Motor leftMotor(MOTOR_L_IN1, MOTOR_L_IN2, PWM_MAX);
Motor rightMotor(MOTOR_R_IN1, MOTOR_R_IN2, PWM_MAX);

// Kinematics
Kinematics kinematics(
    ENCODER_CPR_LEFT,
    ENCODER_CPR_RIGHT,
    WHEEL_RADIUS_M,
    TRACK_WIDTH_M
);

VelocityCmd velocityCmd(
    WHEEL_RADIUS_M,
    TRACK_WIDTH_M
);

WheelPID pidLeft;
WheelPID pidRight;

// IMU (yaw only)
IMU imu(IMU_CS, IMU_INT, IMU_RST);


unsigned long lastControlMs = 0;


void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    delay(1500);

    Serial.println("\n==============================");
    Serial.println(" HAMALS FIRMWARE START");
    Serial.println("==============================");

    leftEncoder.begin();
    rightEncoder.begin();

    leftMotor.begin();
    rightMotor.begin();

    pidLeft.setGains(30.0f, 12.0f, 0.0f);
    pidRight.setGains(30.0f, 12.0f, 0.0f);

    pidLeft.setOutputLimits(-PWM_MAX, PWM_MAX);
    pidRight.setOutputLimits(-PWM_MAX, PWM_MAX);

    pidLeft.reset();
    pidRight.reset();

    if (!imu.begin()) {
        Serial.println("IMU init failed!");
        while (1);
    }
    Serial.println("IMU ready (yaw zeroed)");

    velocityCmd.setTarget(0.2f, 0.0f);   // v = 0.2 m/s, w = 0

    lastControlMs = millis();
}

// --------------------------------------------------
// LOOP
// --------------------------------------------------
void loop() {
    const unsigned long now = millis();
    const float dt = (now - lastControlMs) * 0.001f;

    if (dt < CONTROL_DT_S)
        return;

    lastControlMs = now;

    imu.update();

    velocityCmd.update(dt);

    float omegaL_target = 0.0f;
    float omegaR_target = 0.0f;
    velocityCmd.getWheelTargets(omegaL_target, omegaR_target);


    KinematicsInput kinIn;
    kinIn.delta_left  = leftEncoder.readDelta();
    kinIn.delta_right = rightEncoder.readDelta();
    kinIn.dt          = dt;

    KinematicsOutput kinOut = kinematics.update(kinIn);


    const float pwmL = pidLeft.update(
        omegaL_target,
        kinOut.omega_left,
        dt
    );

    const float pwmR = pidRight.update(
        omegaR_target,
        kinOut.omega_right,
        dt
    );


    leftMotor.setPWM((int)pwmL);
    rightMotor.setPWM((int)pwmR);


    Serial.print("yaw=");
    Serial.print(imu.getYaw(), 3);
    Serial.print(" | v=");
    Serial.print(kinOut.v, 3);
    Serial.print(" w=");
    Serial.print(kinOut.w, 3);

    Serial.print(" | ωL=");
    Serial.print(kinOut.omega_left, 2);
    Serial.print("/");
    Serial.print(omegaL_target, 2);

    Serial.print(" ωR=");
    Serial.print(kinOut.omega_right, 2);
    Serial.print("/");
    Serial.print(omegaR_target, 2);

    Serial.print(" | pwmL=");
    Serial.print(pwmL, 0);
    Serial.print(" pwmR=");
    Serial.println(pwmR, 0);
}