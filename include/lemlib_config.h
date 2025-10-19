/**
 * @file lemlib_config.h
 * @author Push Back Team
 * @brief LemLib configuration for Push Back robot
 * @version 1.0.0
 * @date 2024-01-15
 */

#ifndef _LEMLIB_CONFIG_H_
#define _LEMLIB_CONFIG_H_

#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "config.h"

// =============================================================================
// LEMLIB DRIVETRAIN CONFIGURATION
// =============================================================================

// =============================================================================
// LEMLIB DRIVETRAIN CONFIGURATION
// =============================================================================

// Individual motors
extern pros::Motor left_front_motor;
extern pros::Motor left_middle_motor;
extern pros::Motor left_back_motor;
extern pros::Motor right_front_motor;
extern pros::Motor right_middle_motor;
extern pros::Motor right_back_motor;

// Motor groups
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

// Drivetrain specifications
#define DRIVE_TRACK_WIDTH 13.5      // Distance between left and right wheels (inches)
#define DRIVE_WHEEL_DIAMETER 3.75   // Diameter of drive wheels (inches) - 3.75" omni wheels
#define DRIVE_RPM 360               // Maximum RPM of green cartridge motors

// LemLib drivetrain object
extern lemlib::Drivetrain drivetrain;

// =============================================================================
// LEMLIB TRACKING WHEELS CONFIGURATION  
// =============================================================================

// Tracking wheel specifications
#define TRACKING_WHEEL_DIAMETER 2.75  // Diameter of tracking wheels (inches)
#define VERTICAL_WHEEL_DISTANCE 0     // Distance from center of robot to vertical tracking wheel
#define HORIZONTAL_WHEEL_DISTANCE 5.5 // Distance from center of robot to horizontal tracking wheel

// Tracking wheel objects
extern pros::Rotation vertical_encoder;
extern pros::Rotation horizontal_encoder;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::TrackingWheel horizontal_tracking_wheel;

// =============================================================================
// LEMLIB IMU CONFIGURATION
// =============================================================================

extern pros::Imu inertial_sensor;

// =============================================================================
// LEMLIB PID SETTINGS
// =============================================================================

// Linear movement controller (for driving to points)
extern lemlib::ControllerSettings linear_controller;

// Angular movement controller (for turning)
extern lemlib::ControllerSettings angular_controller;

// =============================================================================
// LEMLIB ODOMETRY SENSORS
// =============================================================================

extern lemlib::OdomSensors odometry_sensors;

// =============================================================================
// LEMLIB DRIVETRAIN AND CHASSIS
// =============================================================================

extern lemlib::Drivetrain lemlib_drivetrain;
extern lemlib::Chassis chassis;

// =============================================================================
// INITIALIZATION FUNCTIONS
// =============================================================================

/**
 * @brief Initialize all LemLib components
 * 
 * Call this in initialize() before using any LemLib functions
 */
void initializeLemLib();

#endif // _LEMLIB_CONFIG_H_