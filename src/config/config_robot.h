#pragma once

// -------------------- Wheel & Encoder ------------------

// Wheel radius in meters
constexpr float WHEEL_RADIUS_M = 0.035f;   // 3.5 cm

// Encoder resolution (counts per motor shaft revolution)
constexpr int ENCODER_CPR_LEFT  = 3950;
constexpr int ENCODER_CPR_RIGHT = 3950;


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

// -------------------- Command Timeout -----------------

// Maximum allowed time without new cmd_vel (seconds)
constexpr float CMD_VEL_GRACE_S   = 0.3f;  // soft zone
constexpr float CMD_VEL_TIMEOUT_S = 0.5f;  // hard stop

// -------------------- PWM Limits ----------------------

// PWM range (platform dependent)
constexpr int PWM_MAX = 255;
constexpr int PWM_MIN = 0;

// -------------------- Motion Limits -------------------

// Maximum wheel angular acceleration (rad/s^2)
constexpr float MAX_WHEEL_ACCEL_RAD_S2 = 50.0f;

// -------------------- Yaw Correction ------------------

// Enable heading hold (yaw correction) when w ≈ 0
// TURN OFF THIS VALUE WHEN NAV2 IS TRUE !!!!!
constexpr bool ENABLE_YAW_CORRECTION = false;

// Only active if |w_target| < threshold
constexpr float YAW_CORRECTION_W_THRESHOLD = 0.05f; // rad/s

// Proportional gain
constexpr float YAW_CORRECTION_KP = 2.0f; // 1.0–3.0

// -------------------- Wheel PID ------------------------

// Wheel speed PID gains
constexpr float WHEEL_PID_KP = 18.0f;
constexpr float WHEEL_PID_KI = 5.0f;
constexpr float WHEEL_PID_KD = 0.0f;

// PWM ramp limit (per control step)
constexpr float WHEEL_PID_RAMP_STEP = 10.0f;

// -------------------- Serial TX -----------------------

// Odometry transmit period (seconds)
constexpr float ODOM_TX_DT_S = 0.02f;      // 50 Hz