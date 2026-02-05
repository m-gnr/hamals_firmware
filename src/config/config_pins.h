// DENEYAP Kart 1A v2 MCU
#pragma once

// -------------------- Encoder Pins ---------------------
// Left wheel encoder
#define ENC_L_A   D13   // D13 - interrupt capable - Channel A
#define ENC_L_B   D14   // D14 - interrupt capable - Channel B

// Right wheel encoder
#define ENC_R_A   D8    // D8  - interrupt capable - Channel A
#define ENC_R_B   D9    // D9  - interrupt capable - Channel B

// -------------------- Motor Driver Pins ----------------
// Left motor driver
#define MOTOR_L_IN1  A1
#define MOTOR_L_IN2  A0

// Right motor driver
#define MOTOR_R_IN1  A2
#define MOTOR_R_IN2  A3

// -------------------- IMU (BNO085 - SPI) ----------------
// SPI interface
#define IMU_CS    4    // Chip Select
#define IMU_INT   0    // Data Ready Interrupt (TODO: move to D15/D16/D17)
#define IMU_RST   12   // Reset

// -------------------- Serial ---------------------------
#define SERIAL_BAUDRATE 115200