/**
 * \file config.h
 *
 * Hardware configuration definitions for the pushback robot.
 * This file contains all motor ports// Indexer motor directions for scoring modes
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
#define TOP_INDEXER_PORT        8   // Top indexer motor (shared: front top OR back top)
#define FRONT_LOADER_MOTOR_PORT 7   // Front match loader motor

// Odometry and navigation sensors
#define VERTICAL_ENCODER_PORT   9   // Vertical tracking wheel encoder
#define HORIZONTAL_ENCODER_PORT 10  // Horizontal tracking wheel encoder  
#define GYRO_PORT              13   // Inertial sensor for heading

// =============================================================================
// ADI PORTS - Sensors and Legacy Devices
// =============================================================================

// Front match loader encoder (VEX shaft encoder)
#define FRONT_LOADER_ENCODER_TOP    'E'  // ADI port E 
#define FRONT_LOADER_ENCODER_BOTTOM 'E'  // ADI port E (same port for shaft encoder)

// =============================================================================

// PTO (Power Take-Off) pneumatic cylinders
// These control whether middle wheels are connected to drivetrain or scorer
#define PTO_LEFT_PNEUMATIC      'A'  // ADI port A (drives both PTOs)
#define PTO_RIGHT_PNEUMATIC     'A'  // ADI port A (same as left - both PTOs)

// Front scoring flap pneumatic
// Controls flap that holds balls for front scoring
#define FRONT_FLAP_PNEUMATIC    'B'  // ADI port B (moved from C)

// =============================================================================
// FRONT MATCH LOADER CONFIGURATION  
// =============================================================================

// Front loader positions (in degrees - measured at the loader arm, not motor)
#define FRONT_LOADER_RETRACTED_POSITION    0     // Retracted/stored position (vertical)
#define FRONT_LOADER_DEPLOYED_POSITION    -66    // Deployed position (close to observed physical limit)

// Front loader motor configuration
#define FRONT_LOADER_MOTOR_SPEED          80     // Motor speed in RPM (reduced to prevent stalling)
#define FRONT_LOADER_POSITION_TOLERANCE   3      // Position tolerance in degrees (increased)
#define FRONT_LOADER_GEAR_RATIO          12.0    // Gear ratio (72 teeth / 6 teeth = 12:1)
#define FRONT_LOADER_REVERSE_MOTOR       true    // Set to true if motor moves in wrong direction

// Position feedback method
#define USE_MOTOR_ENCODER_ONLY           true    // true = motor encoder, false = potentiometer

// Potentiometer configuration
#define POTENTIOMETER_RANGE_DEGREES      270.0   // VEX potentiometer range (270 degrees)
#define POTENTIOMETER_MAX_VALUE          4095    // 12-bit ADC max value
#define POTENTIOMETER_MOUNTED_ON_MOTOR   true    // true = on motor shaft, false = on loader arm

// =============================================================================
// CONTROLLER CONFIGURATION
// =============================================================================

// Tank drive control mapping
#define TANK_DRIVE_LEFT_STICK   pros::E_CONTROLLER_ANALOG_LEFT_Y
#define TANK_DRIVE_RIGHT_STICK  pros::E_CONTROLLER_ANALOG_RIGHT_Y

// NEW CONTROL SCHEME: Two-step scoring system
// Step 1: Mode selection buttons (Y/A/B/X)
#define COLLECTION_MODE_BUTTON     pros::E_CONTROLLER_DIGITAL_Y   // Collection/intake mode
#define MID_GOAL_BUTTON           pros::E_CONTROLLER_DIGITAL_A   // Mid level scoring
#define LOW_GOAL_BUTTON           pros::E_CONTROLLER_DIGITAL_B   // Low goal scoring (intake reverse)
#define TOP_GOAL_BUTTON           pros::E_CONTROLLER_DIGITAL_X   // Top level scoring

// Step 2: Execution buttons (R1/R2)
#define BACK_EXECUTE_BUTTON       pros::E_CONTROLLER_DIGITAL_R1  // Execute selected mode - back
#define FRONT_EXECUTE_BUTTON      pros::E_CONTROLLER_DIGITAL_R2  // Execute selected mode - front

// Additional controls for testing individual indexers
#define LEFT_INDEXER_TEST_BUTTON  pros::E_CONTROLLER_DIGITAL_L1   // Test left indexer
#define RIGHT_INDEXER_TEST_BUTTON pros::E_CONTROLLER_DIGITAL_L2   // Test right indexer

// PTO control (if still needed) - moved to UP button
#define PTO_TOGGLE_BUTTON         pros::E_CONTROLLER_DIGITAL_UP   // PTO toggle (optional)

// Intake mechanism control - DOWN button
#define INTAKE_TOGGLE_BUTTON      pros::E_CONTROLLER_DIGITAL_DOWN // Intake toggle (extend/retract)

// Storage scoring control - LEFT button
#define STORAGE_TOGGLE_BUTTON     pros::E_CONTROLLER_DIGITAL_LEFT // Toggle score from top storage mode

// Front flap direct control - RIGHT button
#define FRONT_FLAP_TOGGLE_BUTTON  pros::E_CONTROLLER_DIGITAL_RIGHT // Toggle front flap open/closed

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
#define PTO_EXTENDED    false   // Extended = drivetrain mode (3-wheel drive)
#define PTO_RETRACTED   true  // Retracted = scorer mode (2-wheel drive, middle wheels for scorer)

// Front flap pneumatic states
#define FRONT_FLAP_OPEN   false  // Open = balls can score through front (reversed due to wiring)
#define FRONT_FLAP_CLOSED true   // Closed = balls held against flap (reversed due to wiring)

// Default PTO state on robot startup
#define PTO_DEFAULT_STATE PTO_EXTENDED

// Default front flap state on robot startup
#define FRONT_FLAP_DEFAULT_STATE FRONT_FLAP_CLOSED

// =============================================================================
// INTAKE MECHANISM CONFIGURATION (FRONT MATCH LOADER)
// =============================================================================

// Front loader states - now using motor positions instead of pneumatics
#define FRONT_LOADER_DEPLOYED   true   // Deployed = loader extended for ball collection  
#define FRONT_LOADER_RETRACTED  false  // Retracted = loader stored (default position)

// Default front loader state on robot startup
#define FRONT_LOADER_DEFAULT_STATE FRONT_LOADER_RETRACTED

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
#define INPUT_MOTOR_REVERSE_SPEED   -120   // Reverse speed for low goal scoring

// Direct indexer motor speeds for each mode (positive/negative for direction control)
// Front scoring speeds (reduced to 50% to prevent overloading)
#define LEFT_INDEXER_FRONT_COLLECTION_SPEED   -50    // Front collection mode (was -100)
#define LEFT_INDEXER_FRONT_MID_GOAL_SPEED     50     // Front mid goal (opposite direction) (was 100)
#define LEFT_INDEXER_FRONT_IMMEDIATE_SPEED     -60    // Front immediate mode (was -120)
#define LEFT_INDEXER_FRONT_TOP_GOAL_SPEED      -75    // Front top goal mode (was -150)

// Back scoring speeds (when left indexer helps back scoring - reduced to 50%)
#define LEFT_INDEXER_BACK_COLLECTION_SPEED     60     // Back collection helper (was 120)
#define LEFT_INDEXER_BACK_MID_GOAL_SPEED      -50     // Back mid goal helper (was -100)
#define LEFT_INDEXER_BACK_IMMEDIATE_SPEED      60     // Back immediate helper (was 120)
#define LEFT_INDEXER_BACK_TOP_GOAL_SPEED       -50    // Back top goal helper (was -100)

// Right indexer speeds (back scoring main motor)
#define RIGHT_INDEXER_COLLECTION_SPEED         -120   // Back collection mode
#define RIGHT_INDEXER_MID_GOAL_SPEED          100   // Back mid goal mode
#define RIGHT_INDEXER_IMMEDIATE_SPEED          -120   // Back immediate mode
#define RIGHT_INDEXER_TOP_GOAL_SPEED          -150   // Back top goal mode

// Top indexer speeds
#define TOP_INDEXER_FRONT_SPEED               150   // Top indexer when scoring front
#define TOP_INDEXER_BACK_SPEED                -150   // Top indexer when scoring back (opposite)

// Storage mode speeds - for moving balls from top storage back toward intake
#define TOP_INDEXER_STORAGE_SPEED             -120   // Top indexer moves balls back from storage
#define FRONT_INDEXER_STORAGE_SPEED           60     // Front indexer moves balls back from storage (was 120)

// =============================================================================
// AUTONOMOUS SYSTEM CONFIGURATION
// =============================================================================

// Odometry wheel specifications (adjust based on your actual wheels)
#define TRACKING_WHEEL_DIAMETER  2.75  // Diameter of tracking wheels in inches
#define TRACKING_WHEEL_CIRCUMFERENCE (TRACKING_WHEEL_DIAMETER * M_PI)

// Robot dimensions (adjust based on your actual robot)
#define ROBOT_WIDTH             15.0   // Distance between left/right wheels (inches)
#define ROBOT_LENGTH            15.0   // Robot length (inches)

// Movement control constants
#define DRIVE_KP                0.8    // Proportional gain for driving
#define DRIVE_KI                0.01   // Integral gain for driving
#define DRIVE_KD                0.1    // Derivative gain for driving
#define TURN_KP                 1.2    // Proportional gain for turning
#define TURN_KI                 0.02   // Integral gain for turning
#define TURN_KD                 0.15   // Derivative gain for turning

// Movement thresholds
#define POSITION_THRESHOLD      2.0    // Acceptable error for position (inches)
#define HEADING_THRESHOLD       2.0    // Acceptable error for heading (degrees)
#define DRIVE_MAX_SPEED         127    // Maximum drive speed
#define TURN_MAX_SPEED          100    // Maximum turn speed

// Autonomous mode enumeration
enum class AutoMode {
    DISABLED = 0,
    RED_LEFT_AWP = 1,
    RED_LEFT_BONUS = 2,
    RED_RIGHT_AWP = 3,
    RED_RIGHT_BONUS = 4,
    SKILLS = 5,
    TEST_DRIVE = 6,
    TEST_TURN = 7,
    TEST_NAVIGATION = 8,
    TEST_ODOMETRY = 9
};

#endif // _CONFIG_H_