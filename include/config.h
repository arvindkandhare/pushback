/**
 * \file config.h
 *
 * Hardware configuration definitions for the pushback robot.
 * This file contains all motor ports, pneumatic ports, and controller mappings.
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "api.h"

// =============================================================================
// MOTOR PORTS - V5 Smart Motors (11W Green Cartridge)
// =============================================================================

// Left side drivetrain motors
#define LEFT_FRONT_MOTOR_PORT   3
#define LEFT_MIDDLE_MOTOR_PORT  4
#define LEFT_BACK_MOTOR_PORT    15

// Right side drivetrain motors  
#define RIGHT_FRONT_MOTOR_PORT  6
#define RIGHT_MIDDLE_MOTOR_PORT 2
#define RIGHT_BACK_MOTOR_PORT   16

// Indexer and intake system motors
#define INPUT_MOTOR_PORT        1   // 11W motor at bottom for ball intake
#define FRONT_INDEXER_PORT      8   // Dedicated motor for front indexer

/* vertical odom 9
horizontal 10
gyro 13
 */

// =============================================================================
// PNEUMATIC PORTS - ADI (Analog/Digital Interface)
// =============================================================================

// PTO (Power Take-Off) pneumatic cylinders
// These control whether middle wheels are connected to drivetrain or scorer
#define PTO_LEFT_PNEUMATIC      'A'  // ADI port A
#define PTO_RIGHT_PNEUMATIC     'B'  // ADI port B

// =============================================================================
// CONTROLLER CONFIGURATION
// =============================================================================

// Tank drive control mapping
#define TANK_DRIVE_LEFT_STICK   pros::E_CONTROLLER_ANALOG_LEFT_Y
#define TANK_DRIVE_RIGHT_STICK  pros::E_CONTROLLER_ANALOG_RIGHT_Y

// PTO control button
#define PTO_TOGGLE_BUTTON       pros::E_CONTROLLER_DIGITAL_R1

// Indexer and scoring system controls
#define INPUT_MOTOR_BUTTON      pros::E_CONTROLLER_DIGITAL_R2  // Ball intake
#define FRONT_SCORING_BUTTON    pros::E_CONTROLLER_DIGITAL_L1  // Select front scoring
#define BACK_SCORING_BUTTON     pros::E_CONTROLLER_DIGITAL_L2  // Select back scoring
#define LONG_GOAL_BUTTON        pros::E_CONTROLLER_DIGITAL_X   // Top scoring (long goal)
#define MID_GOAL_BUTTON         pros::E_CONTROLLER_DIGITAL_B   // Bottom scoring (mid goal)
#define EXECUTE_SCORING_BUTTON  pros::E_CONTROLLER_DIGITAL_A   // Execute scoring sequence

// Additional controls for testing individual indexers
#define LEFT_INDEXER_TEST_BUTTON  pros::E_CONTROLLER_DIGITAL_Y   // Test left indexer
#define RIGHT_INDEXER_TEST_BUTTON pros::E_CONTROLLER_DIGITAL_UP  // Test right indexer

// =============================================================================
// MOTOR CONFIGURATION CONSTANTS
// =============================================================================

// Motor gearset (11W motors use 18:1 green cartridge for good balance of speed/torque)
#define DRIVETRAIN_GEARSET      pros::v5::MotorGears::green

// Motor brake mode (coast allows for easier pushing, brake provides better control)
#define DRIVETRAIN_BRAKE_MODE   pros::v5::MotorBrake::coast

// Reverse motors on left side for proper tank drive operation
#define LEFT_MOTORS_REVERSED    true
#define RIGHT_MOTORS_REVERSED   false

// =============================================================================
// PTO CONFIGURATION
// =============================================================================

// PTO pneumatic states
#define PTO_EXTENDED    true   // Extended = drivetrain mode (3-wheel drive)
#define PTO_RETRACTED   false  // Retracted = scorer mode (2-wheel drive, middle wheels for scorer)

// Default PTO state on robot startup
#define PTO_DEFAULT_STATE PTO_EXTENDED

// =============================================================================
// DRIVE CONFIGURATION
// =============================================================================

// Tank drive sensitivity (0.0 to 1.0)
#define TANK_DRIVE_SENSITIVITY  1.0

// Deadzone for joysticks (prevents drift)
#define JOYSTICK_DEADZONE      10

// Maximum motor velocity (RPM) - 11W motors max ~200 RPM with 18:1 gearing
#define MAX_DRIVE_VELOCITY     200

// =============================================================================
// INDEXER SYSTEM CONFIGURATION
// =============================================================================

// Indexer motor speeds (RPM)
#define INDEXER_SPEED_TOP_SCORING    150   // Speed for top scoring (long goal)
#define INDEXER_SPEED_BOTTOM_SCORING 100   // Speed for bottom scoring (mid goal)
#define INPUT_MOTOR_SPEED            120   // Speed for ball intake

// Indexer motor directions for scoring modes
#define FRONT_INDEXER_TOP_DIRECTION     1   // Positive for top scoring
#define FRONT_INDEXER_BOTTOM_DIRECTION -1   // Negative for bottom scoring
#define BACK_INDEXER_TOP_DIRECTION      1   // Positive for top scoring  
#define BACK_INDEXER_BOTTOM_DIRECTION  -1   // Negative for bottom scoring

// Scoring sequence timing (milliseconds)
#define SCORING_SEQUENCE_DURATION    2000   // How long to run indexers when scoring
#define INPUT_MOTOR_TIMEOUT          5000   // Max time input motor can run continuously

#endif // _CONFIG_H_