// DENEYAP Kart 1A v2 MCU
#pragma once

// -------------------- Encoder Pins ---------------------
// Left wheel encoder
#define ENC_L_A   D12   // GPIO10 - interrupt capable - Channel A
#define ENC_L_B   D11   // GPIO21 - interrupt capable - Channel B

// Right wheel encoder
#define ENC_R_A   D13   // GPIO3  - interrupt capable - Channel A
#define ENC_R_B   D14   // GPIO8  - interrupt capable - Channel B

// -------------------- Motor Driver Pins ----------------
// Left motor driver
#define MOTOR_L_IN1  A1  // GPIO5
#define MOTOR_L_IN2  A0  // GPIO4

// Right motor driver
#define MOTOR_R_IN1  A2  // GPIO6
#define MOTOR_R_IN2  A3  // GPIO7

// -------------------- IMU (BNO085 - SPI) ----------------
#define IMU_CS    D4    // GPIO42 - Chip Select
#define IMU_INT   D1    // GPIO2  - Data Ready Interrupt
#define IMU_RST   D8    // GPIO38 - Reset

// -------------------- Serial ---------------------------
#define SERIAL_BAUDRATE 115200
