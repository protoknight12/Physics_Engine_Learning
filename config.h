#pragma once

// Central simulation constants. Include this header anywhere these values are
// needed (especially headers that declare fixed-size arrays).

inline constexpr int RIGIDBODY_COUNT = 64;
inline constexpr int MAX_POLYGON_VERTICES = 32;
inline constexpr int DEFAULT_POLYGON_SIDES = 6;

// Screen-space gravity in pixels/s^2 (Y increases downward).
inline constexpr double GRAVITY = 400.0;

inline constexpr double MIN_COLLIDER_SIZE = 40.0;
inline constexpr double MAX_COLLIDER_SIZE = 70.0;

inline constexpr double REST_VELOCITY_THRESHOLD = 20.0;
inline constexpr double LINEAR_SLEEP_THRESHOLD_SQ = 0.05;
inline constexpr double ANGULAR_SLEEP_THRESHOLD = 0.01;
inline constexpr double RESTING_CONTACT_NORMAL_Y = 0.5;
inline constexpr double RESTING_CONTACT_VELOCITY = 8.0;
inline constexpr double SLEEP_LINEAR_THRESHOLD_SQ = 4.0;
inline constexpr double SLEEP_ANGULAR_THRESHOLD = 0.08;
inline constexpr int SLEEP_FRAME_THRESHOLD = 20;
inline constexpr double WAKE_LINEAR_THRESHOLD_SQ = 36.0;
inline constexpr double WAKE_ANGULAR_THRESHOLD = 0.2;
inline constexpr double SLEEP_POSITION_SLOP = 0.5;
inline constexpr double RESTING_ANGULAR_LOCK_THRESHOLD = 0.25;

inline constexpr double COLLISION_SLOP = 0.01;
inline constexpr double POSITION_CORRECTION_PERCENT = 0.4;
