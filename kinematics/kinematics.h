#pragma once

#include <stdint.h>

// =======================================================
// Kinematics interface (differential drive)
// -------------------------------------------------------
// Responsibility:
//  - Convert encoder pulse deltas into physical quantities
//  - Wheel angular speed (rad/s)
//  - Robot linear and angular velocity (v, w)
// =======================================================

// -------------------- Input ----------------------------

struct KinematicsInput {
    int32_t delta_left;     // encoder pulse delta (left wheel)
    int32_t delta_right;    // encoder pulse delta (right wheel)
    float   dt;             // time delta in seconds
};

// -------------------- Output ---------------------------

struct KinematicsOutput {
    float omega_left;       // left wheel angular velocity (rad/s)
    float omega_right;      // right wheel angular velocity (rad/s)
    float v;                // robot linear velocity (m/s)
    float w;                // robot angular velocity (rad/s)
};

// -------------------- Kinematics -----------------------

class Kinematics {
public:
    // encoder_cpr : pulses per one full wheel revolution
    // wheel_radius: wheel radius in meters
    // track_width : distance between left and right wheels (meters)
    Kinematics(int32_t encoder_cpr,
               float   wheel_radius,
               float   track_width);

    // Compute kinematics from encoder deltas
    KinematicsOutput update(const KinematicsInput& input);

private:
    int32_t encoder_cpr_;
    float   wheel_radius_;
    float   track_width_;
};
