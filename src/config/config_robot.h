#pragma once
// -------------------- Wheel & Encoder ------------------

// Wheel radius in meters
constexpr float WHEEL_RADIUS_M = 0.035f;   // 3.5 cm

// Encoder resolution (counts per motor shaft revolution)
constexpr int ENCODER_CPR = 1320;           // JGB37-520 @110RPM (11 PPR × 4 × ~30:1) — !! VERIFY BY TEST !!

// Gear reduction ratio (motor → wheel)
constexpr float GEAR_RATIO = 30.0f;          // JGB37-520 gearbox ratio — VERIFY BY TEST

// DEBUG NOTE (Encoder Validation):
// - Rotate ONE wheel exactly 1 full revolution by hand
// - Count encoder pulses reported by firmware
// - Expected value ≈ ENCODER_CPR (±5%)
// - If value differs, update ENCODER_CPR and/or GEAR_RATIO here

// -------------------- Robot Geometry -------------------

// Distance between left and right wheels (meters)
constexpr float TRACK_WIDTH_M = 0.18f;     // 18.0 cm 

// -------------------- Motion Limits -------------------

// Maximum wheel angular speed (rad/s)
constexpr float MAX_WHEEL_RAD_S = 12.0f;

// Maximum linear speed of robot (m/s)
constexpr float MAX_LINEAR_M_S = 0.5f;

// Maximum angular speed of robot (rad/s)
constexpr float MAX_ANGULAR_RAD_S = 1.5f;

// -------------------- Control Timing ------------------

// Control loop period (seconds)
// Must match timing scheduler
constexpr float CONTROL_DT_S = 0.01f;      // 10 ms

// -------------------- PWM Limits ----------------------

// PWM range (platform dependent)
constexpr int PWM_MAX = 255;
constexpr int PWM_MIN = 0;

// -------------------- Motion Limits -------------------

// Maximum wheel angular acceleration (rad/s^2)
constexpr float MAX_WHEEL_ACCEL_RAD_S2 = 5.0f;