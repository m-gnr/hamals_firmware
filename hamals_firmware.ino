#include "src/motor/motor.h"
#include "src/control/wheel_pid.h"
#include "src/control/velocity_cmd.h"
#include "src/config/config_pins.h"
#include "src/config/config_robot.h"

Motor leftMotor(MOTOR_L_IN1, MOTOR_L_IN2, PWM_MAX);
Motor rightMotor(MOTOR_R_IN1, MOTOR_R_IN2, PWM_MAX);

WheelPID pidL;
WheelPID pidR;

VelocityCmd velocityCmd(WHEEL_RADIUS_M, TRACK_WIDTH_M);

void setup() {
    leftMotor.begin();
    rightMotor.begin();

    pidL.setGains(10.0, 0.0, 0.0);
    pidR.setGains(10.0, 0.0, 0.0);
}

void loop() {
    velocityCmd.setTarget(0.2, 0.0);
    velocityCmd.update(CONTROL_DT_S);

    float wL, wR;
    velocityCmd.getWheelTargets(wL, wR);

    float pwmL = pidL.update(wL, 0.0, CONTROL_DT_S);
    float pwmR = pidR.update(wR, 0.0, CONTROL_DT_S);

    leftMotor.setPWM((int)pwmL);
    rightMotor.setPWM((int)pwmR);

    delay(10);
}