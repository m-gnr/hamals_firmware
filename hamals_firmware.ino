#include <Arduino.h>

#include "config/config_pins.h"
#include "config/config_robot.h"

#include "control/velocity_cmd.h"
#include "control/wheel_pid.h"
#include "motor/motor.h"

// -------------------- Objects -------------------------

VelocityCmd velocityCmd(WHEEL_RADIUS_M, TRACK_WIDTH_M);

WheelPID pidLeft;
WheelPID pidRight;

Motor leftMotor(MOTOR_L_IN1, MOTOR_L_IN2, PWM_MAX);
Motor rightMotor(MOTOR_R_IN1, MOTOR_R_IN2, PWM_MAX);

// -------------------- Timing --------------------------

unsigned long last_control_ms = 0;

// -------------------- Setup ---------------------------

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);

    // Motor init
    leftMotor.begin();
    rightMotor.begin();

    // PID setup (çok konservatif başlangıç)
    pidLeft.setGains(20.0f, 0.0f, 0.0f);
    pidRight.setGains(20.0f, 0.0f, 0.0f);

    pidLeft.setOutputLimits(-PWM_MAX, PWM_MAX);
    pidRight.setOutputLimits(-PWM_MAX, PWM_MAX);

    pidLeft.reset();
    pidRight.reset();

    // İlk hedef: yavaşça ileri
    velocityCmd.setTarget(0.2f, 0.0f); // 0.2 m/s düz ileri

    last_control_ms = millis();

    Serial.println("=== DUMMY CONTROL TEST STARTED ===");
}

// -------------------- Loop ----------------------------

void loop()
{
    unsigned long now = millis();

    if (now - last_control_ms >= (unsigned long)(CONTROL_DT_S * 1000.0f)) {
        last_control_ms = now;

        const float dt = CONTROL_DT_S;

        // 1️⃣ Hedefleri ramp ile güncelle
        velocityCmd.update(dt);

        float omegaL_target = 0.0f;
        float omegaR_target = 0.0f;

        velocityCmd.getWheelTargets(omegaL_target, omegaR_target);

        // 2️⃣ DUMMY ölçüm (encoder yok → 0 kabul ediyoruz)
        float omegaL_measured = 0.0f;
        float omegaR_measured = 0.0f;

        // 3️⃣ PID → PWM
        int pwmL = (int)pidLeft.update(
            omegaL_target,
            omegaL_measured,
            dt
        );

        int pwmR = (int)pidRight.update(
            omegaR_target,
            omegaR_measured,
            dt
        );

        // 4️⃣ Motorlara uygula
        leftMotor.setPWM(pwmL);
        rightMotor.setPWM(pwmR);

        // 5️⃣ Debug çıktısı
        Serial.print("omegaL_t: ");
        Serial.print(omegaL_target, 2);
        Serial.print(" | pwmL: ");
        Serial.print(pwmL);

        Serial.print(" || omegaR_t: ");
        Serial.print(omegaR_target, 2);
        Serial.print(" | pwmR: ");
        Serial.println(pwmR);
    }
}