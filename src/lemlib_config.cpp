/**
 * @file lemlib_config.cpp
 * @author Push Back Team  
 * @brief LemLib configuration implementation for Push Back robot
 * @version 1.0.0
 * @date 2024-01-15
 */

#include "lemlib_config.h"

// =============================================================================
// DRIVETRAIN MOTOR CONFIGURATION
// =============================================================================

// =============================================================================
// DRIVETRAIN CONFIGURATION
// =============================================================================

// Create individual motors using PROS 4.2.1 API with proper reversing and BLUE cartridges
pros::Motor left_front_motor(LEFT_MOTORS_REVERSED ? -LEFT_FRONT_MOTOR_PORT : LEFT_FRONT_MOTOR_PORT, pros::v5::MotorGears::blue);
pros::Motor left_middle_motor(LEFT_MOTORS_REVERSED ? -LEFT_MIDDLE_MOTOR_PORT : LEFT_MIDDLE_MOTOR_PORT, pros::v5::MotorGears::blue);
pros::Motor left_back_motor(LEFT_MOTORS_REVERSED ? -LEFT_BACK_MOTOR_PORT : LEFT_BACK_MOTOR_PORT, pros::v5::MotorGears::blue);

pros::Motor right_front_motor(RIGHT_MOTORS_REVERSED ? -RIGHT_FRONT_MOTOR_PORT : RIGHT_FRONT_MOTOR_PORT, pros::v5::MotorGears::blue);
pros::Motor right_middle_motor(RIGHT_MOTORS_REVERSED ? -RIGHT_MIDDLE_MOTOR_PORT : RIGHT_MIDDLE_MOTOR_PORT, pros::v5::MotorGears::blue);
pros::Motor right_back_motor(RIGHT_MOTORS_REVERSED ? -RIGHT_BACK_MOTOR_PORT : RIGHT_BACK_MOTOR_PORT, pros::v5::MotorGears::blue);

// Create motor groups for LemLib (front and back wheels only for consistent drive)
// Middle wheels will be managed separately due to PTO system
// Apply proper motor reversal for LemLib motor groups
pros::MotorGroup left_motor_group({
    LEFT_MOTORS_REVERSED ? -LEFT_FRONT_MOTOR_PORT : LEFT_FRONT_MOTOR_PORT,
    LEFT_MOTORS_REVERSED ? -LEFT_BACK_MOTOR_PORT : LEFT_BACK_MOTOR_PORT
});

pros::MotorGroup right_motor_group({
    RIGHT_MOTORS_REVERSED ? -RIGHT_FRONT_MOTOR_PORT : RIGHT_FRONT_MOTOR_PORT,
    RIGHT_MOTORS_REVERSED ? -RIGHT_BACK_MOTOR_PORT : RIGHT_BACK_MOTOR_PORT
});

// Drivetrain configuration - WORKING VALUES from working_code.txt
lemlib::Drivetrain drivetrain(
    &left_motor_group,
    &right_motor_group,
    DRIVE_TRACK_WIDTH,     // 12.5 inches - PROVEN WORKING
    DRIVE_WHEEL_DIAMETER,  // 3.25 inches - NEW_325 omni wheels  
    DRIVE_RPM,             // 450 RPM - blue cartridge motors
    2.0f // horizontalDrift - 2 for omni wheels (from working code)
);

// =============================================================================
// TRACKING WHEELS CONFIGURATION
// =============================================================================

// Rotation sensors for tracking wheels (using your existing ports)
pros::Rotation vertical_encoder(VERTICAL_ENCODER_PORT);     // Port 9
pros::Rotation horizontal_encoder(HORIZONTAL_ENCODER_PORT); // Port 10

// Tracking wheel objects - WORKING VALUES from working_code.txt
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, 
                                              lemlib::Omniwheel::NEW_2,  // 2.75" tracking wheels
                                              VERTICAL_WHEEL_DISTANCE);   // 3.3215 offset

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                lemlib::Omniwheel::NEW_2,  // 2.75" tracking wheels
                                                HORIZONTAL_WHEEL_DISTANCE); // 4.273 offset

// =============================================================================
// IMU CONFIGURATION
// =============================================================================

// Inertial sensor (using your existing port)
pros::Imu inertial_sensor(GYRO_PORT); // Port 13

// =============================================================================
// PID CONTROLLER SETTINGS - WORKING VALUES from working_code.txt
// =============================================================================

// Linear controller (for driving to points) - PROVEN WORKING
lemlib::ControllerSettings linear_controller(
    DRIVE_KP,              // 20.0 - kP (was 10)
    DRIVE_KI,              // 0.0  - kI  
    DRIVE_KD,              // 110.0 - kD (was 3)
    DRIVE_WINDUP,          // 0 - windupRange
    DRIVE_SMALL_ERROR,     // 0.25 - smallError (inches)
    DRIVE_SMALL_TIMEOUT,   // 10 - smallErrorTimeout (ms)
    DRIVE_LARGE_ERROR,     // 0.5 - largeError (inches)
    DRIVE_LARGE_TIMEOUT,   // 50 - largeErrorTimeout (ms)  
    DRIVE_SLEW             // 1 - maxSpeed (slew)
);

// Angular controller (for turning) - PROVEN WORKING
lemlib::ControllerSettings angular_controller(
    TURN_KP,               // 2.0 - kP
    TURN_KI,               // 0.0 - kI
    TURN_KD,               // 4.0 - kD (was 10)
    TURN_WINDUP,           // 0 - windupRange
    TURN_SMALL_ERROR,      // 0.2 - smallError (degrees)
    TURN_SMALL_TIMEOUT,    // 10 - smallErrorTimeout (ms)
    TURN_LARGE_ERROR,      // 0.75 - largeError (degrees)
    TURN_LARGE_TIMEOUT,    // 50 - largeErrorTimeout (ms)
    TURN_SLEW              // 0 - maxSpeed (slew)
);

// Angular turn controller for larger turns - PROVEN WORKING
lemlib::ControllerSettings angular_turn_controller(
    TURN_BIG_KP,           // 4.0 - kP for big turns
    TURN_BIG_KI,           // 0.0 - kI
    TURN_BIG_KD,           // 9.0 - kD for big turns
    TURN_WINDUP,           // 0 - windupRange
    TURN_BIG_SMALL_ERROR,  // 0.2 - smallError (degrees)
    TURN_SMALL_TIMEOUT,    // 10 - smallErrorTimeout (ms)
    TURN_BIG_LARGE_ERROR,  // 0.5 - largeError (degrees) 
    TURN_LARGE_TIMEOUT,    // 50 - largeErrorTimeout (ms)
    TURN_SLEW              // 0 - maxSpeed (slew)
);

// Angular short turn controller - PROVEN WORKING  
lemlib::ControllerSettings angular_short_turn_controller(
    TURN_BIG_KP,           // 4.0 - kP
    TURN_BIG_KI,           // 0.0 - kI
    8.0,                   // 8.0 - kD (slightly less than big turns)
    TURN_WINDUP,           // 0 - windupRange
    TURN_BIG_SMALL_ERROR,  // 0.2 - smallError (degrees)
    TURN_SMALL_TIMEOUT,    // 10 - smallErrorTimeout (ms)
    TURN_BIG_LARGE_ERROR,  // 0.5 - largeError (degrees)
    TURN_LARGE_TIMEOUT,    // 50 - largeErrorTimeout (ms)
    TURN_SLEW              // 0 - maxSpeed (slew)
);

// =============================================================================
// ODOMETRY SENSORS CONFIGURATION
// =============================================================================

lemlib::OdomSensors odometry_sensors(
    &vertical_tracking_wheel, // vertical1
    nullptr,                 // vertical2
    &horizontal_tracking_wheel, // horizontal1
    nullptr,                 // horizontal2
    &inertial_sensor         // imu
);

// =============================================================================
// LEMLIB CHASSIS CONFIGURATION
// =============================================================================

// LemLib chassis object - this is the main object you'll use for movement
lemlib::Chassis chassis(drivetrain,
                       linear_controller,
                       angular_controller,
                       odometry_sensors);

// Additional chassis objects for different turn scenarios (like working_code.txt)
lemlib::Chassis chassis_turn(drivetrain,
                            linear_controller, 
                            angular_turn_controller,
                            odometry_sensors);

lemlib::Chassis chassis_turn_short(drivetrain,
                                  linear_controller,
                                  angular_short_turn_controller, 
                                  odometry_sensors);

// =============================================================================
// INITIALIZATION FUNCTION
// =============================================================================

void initializeLemLib() {
    printf("Initializing LemLib...\n");
    
    // Calibrate the chassis (this will calibrate IMU and start odometry)
    chassis.calibrate();
    
    printf("LemLib initialization complete!\n");
}