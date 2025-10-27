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

// Drivetrain specifications - WORKING VALUES from working_code.txt
#define DRIVE_TRACK_WIDTH 12.5      // Distance between left and right wheels (inches) - PROVEN WORKING
#define DRIVE_WHEEL_DIAMETER 3.25   // Diameter of drive wheels (inches) - 3.25" omni wheels (NEW_325)
#define DRIVE_RPM 450               // Maximum RPM of blue cartridge motors at 6:1 ratio

// LemLib drivetrain object
extern lemlib::Drivetrain drivetrain;

// =============================================================================
// LEMLIB TRACKING WHEELS CONFIGURATION  
// =============================================================================

// Tracking wheel specifications - WORKING VALUES from working_code.txt  
#define TRACKING_WHEEL_DIAMETER 2.75  // Diameter of tracking wheels (inches) - NEW_2 omni wheels
#define VERTICAL_WHEEL_DISTANCE 3.3215 // Distance from center of robot to vertical tracking wheel (PROVEN)
#define HORIZONTAL_WHEEL_DISTANCE 4.273 // Distance from center of robot to horizontal tracking wheel (PROVEN)

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

// Additional turn controllers for different scenarios (like working_code.txt)
extern lemlib::ControllerSettings angular_turn_controller;     // For larger turns
extern lemlib::ControllerSettings angular_short_turn_controller; // For short turns

// =============================================================================
// LEMLIB ODOMETRY SENSORS
// =============================================================================

extern lemlib::OdomSensors odometry_sensors;

// =============================================================================
// LEMLIB DRIVETRAIN AND CHASSIS
// =============================================================================

extern lemlib::Drivetrain lemlib_drivetrain;
extern lemlib::Chassis chassis;

// Additional chassis objects for different turn scenarios (like working_code.txt)
extern lemlib::Chassis chassis_turn;        // For larger turns
extern lemlib::Chassis chassis_turn_short;  // For short turns

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