#include "kinematics.h"
#include <cmath>

// =======================================================
// Kinematics implementation (differential drive)
// =======================================================

Kinematics::Kinematics(int32_t encoder_cpr,
                       float   wheel_radius,
                       float   track_width)
    : encoder_cpr_(encoder_cpr),
      wheel_radius_(wheel_radius),
      track_width_(track_width) {}

// -------------------------------------------------------
// update()
// -------------------------------------------------------
// Converts encoder pulse deltas into physical velocities.
//
// input.delta_left  -> pulse delta (left wheel)
// input.delta_right -> pulse delta (right wheel)
// input.dt          -> time delta (seconds)
//
// Returns:
//  - wheel angular velocities (rad/s)
//  - robot linear velocity v (m/s)
//  - robot angular velocity w (rad/s)
// -------------------------------------------------------

KinematicsOutput Kinematics::update(const KinematicsInput& input) {
    KinematicsOutput out{};

    // Protect against invalid dt
    if (input.dt <= 0.0f) {
        out.omega_left  = 0.0f;
        out.omega_right = 0.0f;
        out.v           = 0.0f;
        out.w           = 0.0f;
        return out;
    }

    // --------------------
    // Pulse -> wheel angle
    // --------------------
    const float pulses_to_rad = (2.0f * static_cast<float>(M_PI)) /
                                static_cast<float>(encoder_cpr_);

    const float angle_left  =
        static_cast<float>(input.delta_left)  * pulses_to_rad;

    const float angle_right =
        static_cast<float>(input.delta_right) * pulses_to_rad;

    // --------------------
    // Wheel angular speed
    // --------------------
    out.omega_left  = angle_left  / input.dt;  // rad/s
    out.omega_right = angle_right / input.dt;  // rad/s

    // --------------------
    // Wheel linear speed
    // --------------------
    const float v_left  = out.omega_left  * wheel_radius_;
    const float v_right = out.omega_right * wheel_radius_;

    // --------------------
    // Robot velocities
    // --------------------
    out.v = (v_left + v_right) * 0.5f;                 // m/s
    out.w = (v_right - v_left) / track_width_;         // rad/s

    return out;
}
