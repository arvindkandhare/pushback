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
#define VERTICAL_ENCODER_PORT   -9  // Vertical tracking wheel encoder (REVERSED like working code)
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

// Additional controls for testing individual indexers (NOTE: L1/L2 now used for front loader fine adjustment)
#define LEFT_INDEXER_TEST_BUTTON  pros::E_CONTROLLER_DIGITAL_L1   // Test left indexer (OVERRIDDEN by front loader)
#define RIGHT_INDEXER_TEST_BUTTON pros::E_CONTROLLER_DIGITAL_L2   // Test right indexer (OVERRIDDEN by front loader)

// PTO control (if still needed) - moved to UP button
#define PTO_TOGGLE_BUTTON         pros::E_CONTROLLER_DIGITAL_UP   // PTO toggle (optional)

// Intake mechanism control - DOWN button
#define INTAKE_TOGGLE_BUTTON      pros::E_CONTROLLER_DIGITAL_DOWN // Intake toggle (deploy/retract to preset positions)

// Front loader fine adjustment controls (overrides indexer testing)
// L1: Adjust front loader position +1 degree
// L2: Adjust front loader position -1 degree
// DOWN: Reset to original deployed/retracted position (toggles between presets)

// Storage scoring control - LEFT button
#define STORAGE_TOGGLE_BUTTON     pros::E_CONTROLLER_DIGITAL_LEFT // Toggle score from top storage mode

// Front flap direct control - RIGHT button
#define FRONT_FLAP_TOGGLE_BUTTON  pros::E_CONTROLLER_DIGITAL_RIGHT // Toggle front flap open/closed

// =============================================================================
// MOTOR CONFIGURATION CONSTANTS
// =============================================================================

// Motor gearset (11W motors use 6:1 blue cartridge for speed - matches working config)
#define DRIVETRAIN_GEARSET      pros::v5::MotorGears::blue

// Motor brake mode (coast allows for easier pushing, brake provides better control)
#define DRIVETRAIN_BRAKE_MODE   pros::v5::MotorBrake::coast

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

// Indexer motor speeds using move() function (range: -127 to 127, where 127 = 100% power)
// This is much simpler than RPM conversion and directly represents percentage power

// INPUT MOTOR (intake) speeds
#define INPUT_MOTOR_SPEED               125     
#define INPUT_MOTOR_REVERSE_SPEED      -100   

// FRONT INDEXER speeds (optimized per user request)
#define LEFT_INDEXER_FRONT_COLLECTION_SPEED     -100     // 100% power for collection
#define LEFT_INDEXER_FRONT_MID_GOAL_SPEED        80     // 50% power for mid back
#define LEFT_INDEXER_FRONT_TOP_GOAL_SPEED       -100     // 100% power for mid front (used for front top scoring)

// BACK INDEXER speeds (when left indexer helps back scoring)
#define LEFT_INDEXER_BACK_COLLECTION_SPEED       63     // Back collection helper
#define LEFT_INDEXER_BACK_MID_GOAL_SPEED        -80     // Back mid goal helper
#define LEFT_INDEXER_BACK_IMMEDIATE_SPEED       80      // Back immediate helper
#define LEFT_INDEXER_BACK_TOP_GOAL_SPEED        -100     // Back top goal helper 
#define RIGHT_INDEXER_COLLECTION_SPEED         -100    // Back collection mode
#define RIGHT_INDEXER_MID_GOAL_SPEED           100     // Back mid goal mode  
#define RIGHT_INDEXER_IMMEDIATE_SPEED          -125    // Back immediate mode
#define RIGHT_INDEXER_TOP_GOAL_SPEED           -127    // Back top goal mode

// TOP INDEXER speeds
#define TOP_INDEXER_FRONT_SPEED                125    // Top indexer when scoring front
#define TOP_INDEXER_BACK_SPEED                -125    // Top indexer when scoring back (opposite)

// STORAGE MODE speeds - for moving balls from top storage toward goals
#define TOP_INDEXER_STORAGE_TO_FRONT_SPEED     125     // Top indexer moves balls from storage toward front goal (max speed)
#define TOP_INDEXER_STORAGE_TO_BACK_SPEED     -125     // Top indexer moves balls from storage toward back goal (max speed)
#define FRONT_INDEXER_STORAGE_SPEED            100     // Front indexer moves balls back from storage

// =============================================================================
// AUTONOMOUS SYSTEM CONFIGURATION
// =============================================================================

// Odometry wheel specifications (adjust based on your actual wheels)
#define TRACKING_WHEEL_DIAMETER  2.0  // Diameter of tracking wheels in inches (actual 2.0" wheels)
#define TRACKING_WHEEL_CIRCUMFERENCE (TRACKING_WHEEL_DIAMETER * M_PI)

// Robot dimensions (adjust based on your actual robot)
#define ROBOT_WIDTH             15.0   // Distance between left/right wheels (inches)
#define ROBOT_LENGTH            15.0   // Robot length (inches)

// Movement control constants - WORKING VALUES from working_code.txt
// Linear PID (for driving to points) - PROVEN WORKING
#define DRIVE_KP                20.0   // Proportional gain for driving - PROVEN WORKING VALUE
#define DRIVE_KI                0.0    // Integral gain for driving
#define DRIVE_KD                110.0  // Derivative gain for driving - PROVEN WORKING VALUE
#define DRIVE_WINDUP            0      // Anti windup
#define DRIVE_SMALL_ERROR       0.25   // Small error range (inches) - PROVEN WORKING
#define DRIVE_SMALL_TIMEOUT     10     // Small error timeout (ms) - PROVEN WORKING
#define DRIVE_LARGE_ERROR       0.5    // Large error range (inches) - PROVEN WORKING
#define DRIVE_LARGE_TIMEOUT     50     // Large error timeout (ms) - PROVEN WORKING
#define DRIVE_SLEW              1      // Maximum acceleration - PROVEN WORKING

// Angular PID (for turning) - PROVEN WORKING VALUES
#define TURN_KP                 2.0    // Proportional gain for turning - PROVEN WORKING
#define TURN_KI                 0.0    // Integral gain for turning
#define TURN_KD                 4.0    // Derivative gain for turning - PROVEN WORKING
#define TURN_WINDUP             0      // Anti windup
#define TURN_SMALL_ERROR        0.2    // Small error range (degrees) - PROVEN WORKING
#define TURN_SMALL_TIMEOUT      10     // Small error timeout (ms) - PROVEN WORKING  
#define TURN_LARGE_ERROR        0.75   // Large error range (degrees) - PROVEN WORKING
#define TURN_LARGE_TIMEOUT      50     // Large error timeout (ms) - PROVEN WORKING
#define TURN_SLEW               0      // Maximum acceleration - PROVEN WORKING

// Separate turn controller for larger turns - PROVEN WORKING VALUES
#define TURN_BIG_KP             4.0    // Proportional gain for big turns - PROVEN WORKING
#define TURN_BIG_KI             0.0    // Integral gain for big turns
#define TURN_BIG_KD             9.0    // Derivative gain for big turns - PROVEN WORKING
#define TURN_BIG_SMALL_ERROR    0.2    // Small error range (degrees) - PROVEN WORKING
#define TURN_BIG_LARGE_ERROR    0.5    // Large error range (degrees) - PROVEN WORKING

// Movement thresholds
#define POSITION_THRESHOLD      2.0    // Acceptable error for position (inches)
#define HEADING_THRESHOLD       2.0    // Acceptable error for heading (degrees)
#define DRIVE_MAX_SPEED         127    // Maximum drive speed
#define TURN_MAX_SPEED          100    // Maximum turn speed

// Autonomous mode enumeration
enum class AutoMode {
    DISABLED = 0,
    
    // Bonus Point Routes (Primary Strategy)
    RED_LEFT_BONUS = 1,
    RED_RIGHT_BONUS = 2, 
    BLUE_LEFT_BONUS = 3,
    BLUE_RIGHT_BONUS = 4,
    
    // AWP Routes (Backup Strategy)
    RED_LEFT_AWP = 5,
    RED_RIGHT_AWP = 6,
    BLUE_LEFT_AWP = 7, 
    BLUE_RIGHT_AWP = 8,
    
    SKILLS = 9,
    TEST_DRIVE = 10,
    TEST_TURN = 11,
    TEST_NAVIGATION = 12,
    TEST_ODOMETRY = 13,
    TEST_MOTORS = 14
};

#endif // _CONFIG_H_