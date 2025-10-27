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

// Create individual motors using PROS 4.2.1 API with proper reversing
pros::Motor left_front_motor(LEFT_MOTORS_REVERSED ? -LEFT_FRONT_MOTOR_PORT : LEFT_FRONT_MOTOR_PORT, pros::v5::MotorGears::green);
pros::Motor left_middle_motor(LEFT_MOTORS_REVERSED ? -LEFT_MIDDLE_MOTOR_PORT : LEFT_MIDDLE_MOTOR_PORT, pros::v5::MotorGears::green);
pros::Motor left_back_motor(LEFT_MOTORS_REVERSED ? -LEFT_BACK_MOTOR_PORT : LEFT_BACK_MOTOR_PORT, pros::v5::MotorGears::green);

pros::Motor right_front_motor(RIGHT_MOTORS_REVERSED ? -RIGHT_FRONT_MOTOR_PORT : RIGHT_FRONT_MOTOR_PORT, pros::v5::MotorGears::green);
pros::Motor right_middle_motor(RIGHT_MOTORS_REVERSED ? -RIGHT_MIDDLE_MOTOR_PORT : RIGHT_MIDDLE_MOTOR_PORT, pros::v5::MotorGears::green);
pros::Motor right_back_motor(RIGHT_MOTORS_REVERSED ? -RIGHT_BACK_MOTOR_PORT : RIGHT_BACK_MOTOR_PORT, pros::v5::MotorGears::green);

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

// Drivetrain configuration
lemlib::Drivetrain drivetrain(
    &left_motor_group,
    &right_motor_group,
    DRIVE_TRACK_WIDTH,
    DRIVE_WHEEL_DIAMETER,
    DRIVE_RPM,
    0.0f // horizontalDrift
);

// =============================================================================
// TRACKING WHEELS CONFIGURATION
// =============================================================================

// Rotation sensors for tracking wheels (using your existing ports)
pros::Rotation vertical_encoder(VERTICAL_ENCODER_PORT);     // Port 9
pros::Rotation horizontal_encoder(HORIZONTAL_ENCODER_PORT); // Port 10

// Tracking wheel objects
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, 
                                              TRACKING_WHEEL_DIAMETER, 
                                              VERTICAL_WHEEL_DISTANCE);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                TRACKING_WHEEL_DIAMETER,
                                                HORIZONTAL_WHEEL_DISTANCE);

// =============================================================================
// IMU CONFIGURATION
// =============================================================================

// Inertial sensor (using your existing port)
pros::Imu inertial_sensor(GYRO_PORT); // Port 13

// =============================================================================
// PID CONTROLLER SETTINGS
// =============================================================================

// Linear controller (for driving to points)
lemlib::ControllerSettings linear_controller(
    10,    // kP
    0,     // kI
    3,     // kD
    0,     // windupRange
    1,     // smallError
    100,   // smallErrorTimeout
    3,     // largeError
    500,   // largeErrorTimeout
    110    // maxSpeed (slew)
);

// Angular controller (for turning)
lemlib::ControllerSettings angular_controller(
    2,     // kP
    0,     // kI
    10,    // kD
    0,     // windupRange
    1,     // smallError
    100,   // smallErrorTimeout
    3,     // largeError
    500,   // largeErrorTimeout
    90     // maxSpeed (slew)
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

// =============================================================================
// INITIALIZATION FUNCTION
// =============================================================================

void initializeLemLib() {
    printf("Initializing LemLib...\n");
    
    // Calibrate the chassis (this will calibrate IMU and start odometry)
    chassis.calibrate();
    
    printf("LemLib initialization complete!\n");
}