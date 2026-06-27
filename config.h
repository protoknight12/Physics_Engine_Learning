#pragma once

// Central simulation constants. Include this header anywhere these values are
// needed (especially headers that declare fixed-size arrays).

inline constexpr int RIGIDBODY_COUNT = 20;

// Screen-space gravity in pixels/s^2 (Y increases downward).
inline constexpr double GRAVITY = 400.0;

inline constexpr double MIN_COLLIDER_SIZE = 40.0;
inline constexpr double MAX_COLLIDER_SIZE = 70.0;

inline constexpr double REST_VELOCITY_THRESHOLD = 20.0;
inline constexpr double LINEAR_SLEEP_THRESHOLD_SQ = 0.05;
inline constexpr double ANGULAR_SLEEP_THRESHOLD = 0.01;

inline constexpr double COLLISION_SLOP = 0.01;
inline constexpr double POSITION_CORRECTION_PERCENT = 0.4;
