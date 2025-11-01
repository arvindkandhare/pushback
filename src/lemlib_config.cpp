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

// Global pointers for motors (initialized in initializeLemLib())
pros::Motor* left_front_motor = nullptr;
pros::Motor* left_middle_motor = nullptr;
pros::Motor* left_back_motor = nullptr;
pros::Motor* right_front_motor = nullptr;
pros::Motor* right_middle_motor = nullptr;
pros::Motor* right_back_motor = nullptr;

// Motor groups (initialized in initializeLemLib())
pros::MotorGroup* left_motor_group = nullptr;
pros::MotorGroup* right_motor_group = nullptr;

// Drivetrain configuration (initialized in initializeLemLib())
lemlib::Drivetrain* drivetrain = nullptr;

// =============================================================================
// DRIVE CURVES CONFIGURATION
// =============================================================================

// Drive curves for smooth joystick control (initialized in initializeLemLib())
lemlib::ExpoDriveCurve* throttleCurve = nullptr;
lemlib::ExpoDriveCurve* steerCurve = nullptr;

// =============================================================================
// TRACKING WHEELS CONFIGURATION
// =============================================================================

// Rotation sensors for tracking wheels (initialized in initializeLemLib())
pros::Rotation* vertical_encoder = nullptr;
pros::Rotation* horizontal_encoder = nullptr;

// Tracking wheel objects (initialized in initializeLemLib())
lemlib::TrackingWheel* vertical_tracking_wheel = nullptr;
lemlib::TrackingWheel* horizontal_tracking_wheel = nullptr;

// =============================================================================
// IMU CONFIGURATION
// =============================================================================

// Inertial sensor (initialized in initializeLemLib())
pros::Imu* inertial_sensor = nullptr;

// =============================================================================
// PID CONTROLLER SETTINGS - WORKING VALUES from working_code.txt
// =============================================================================

// PID Controllers (initialized in initializeLemLib())
lemlib::ControllerSettings* linear_controller = nullptr;
lemlib::ControllerSettings* angular_controller = nullptr;
lemlib::ControllerSettings* angular_turn_controller = nullptr;
lemlib::ControllerSettings* angular_short_turn_controller = nullptr;
// =============================================================================
// ODOMETRY SENSORS CONFIGURATION - TRACKING WHEELS ENABLED
// =============================================================================

// Odometry sensors (initialized in initializeLemLib())
lemlib::OdomSensors* odometry_sensors = nullptr;

// =============================================================================
// LEMLIB CHASSIS CONFIGURATION
// =============================================================================

// LemLib chassis objects (initialized in initializeLemLib())
lemlib::Drivetrain* lemlib_drivetrain = nullptr;
lemlib::Chassis* chassis = nullptr;
lemlib::Chassis* chassis_turn = nullptr;        
lemlib::Chassis* chassis_turn_short = nullptr;

// =============================================================================
// INITIALIZATION FUNCTION
// =============================================================================

// Static flag to ensure initialization only happens once
static bool lemlib_initialized = false;

void initializeLemLib() {
    // Check if already initialized
    if (lemlib_initialized) {
        printf("⚠️  WARNING: initializeLemLib() already called - skipping duplicate initialization\n");
        printf("✅ LemLib objects already exist and are ready\n");
        return;
    }
    
    printf("Initializing LemLib...\n");
    
    // =============================================================================
    // INITIALIZE MOTORS
    // =============================================================================
    
    printf("Creating motor objects...\n");
    
    // Create individual motors using config.h defines for consistency
    // Individual motors use raw port numbers for consistency
    left_front_motor = new pros::Motor(-LEFT_FRONT_MOTOR_PORT, pros::v5::MotorGears::blue);    // port 3 (reversed)
    left_middle_motor = new pros::Motor(-LEFT_MIDDLE_MOTOR_PORT, pros::v5::MotorGears::blue);   // port 4 (reversed)  
    left_back_motor = new pros::Motor(-LEFT_BACK_MOTOR_PORT, pros::v5::MotorGears::blue);    // port 15 (reversed)

    right_front_motor = new pros::Motor(RIGHT_FRONT_MOTOR_PORT, pros::v5::MotorGears::blue);    // port from config.h
    right_middle_motor = new pros::Motor(RIGHT_MIDDLE_MOTOR_PORT, pros::v5::MotorGears::blue);   // port from config.h
    right_back_motor = new pros::Motor(RIGHT_BACK_MOTOR_PORT, pros::v5::MotorGears::blue);    // port from config.h

    // Create motor groups for LemLib using config.h defines for maintainability
    // Note: We use the RAW port numbers here, and apply negatives to get the reversal
    left_motor_group = new pros::MotorGroup({
        -LEFT_FRONT_MOTOR_PORT,   // (reversed)
        -LEFT_MIDDLE_MOTOR_PORT,  // (reversed)
        -LEFT_BACK_MOTOR_PORT     // (reversed)
    }, pros::v5::MotorGears::blue);

    right_motor_group = new pros::MotorGroup({
        RIGHT_FRONT_MOTOR_PORT,   
        RIGHT_MIDDLE_MOTOR_PORT,  
        RIGHT_BACK_MOTOR_PORT
    }, pros::v5::MotorGears::blue);

    // =============================================================================
    // INITIALIZE TRACKING WHEELS
    // =============================================================================
    
    printf("Creating tracking wheel objects...\n");
    
    // Rotation sensors for tracking wheels
    vertical_encoder = new pros::Rotation(VERTICAL_ENCODER_PORT);
    horizontal_encoder = new pros::Rotation(HORIZONTAL_ENCODER_PORT);

    // Tracking wheel objects - MATCH working code exactly
    vertical_tracking_wheel = new lemlib::TrackingWheel(vertical_encoder, 
                                                      lemlib::Omniwheel::NEW_2,    // 2.125" (closest to your 2.0" wheels)
                                                      VERTICAL_WHEEL_DISTANCE);     // POSITIVE: hardware reversal in port

    horizontal_tracking_wheel = new lemlib::TrackingWheel(horizontal_encoder,
                                                        lemlib::Omniwheel::NEW_2,    // 2.125" (closest to your 2.0" wheels)
                                                        HORIZONTAL_WHEEL_DISTANCE);   // 4.273 offset

    // =============================================================================
    // INITIALIZE IMU
    // =============================================================================
    
    printf("Creating IMU object...\n");
    
    // Inertial sensor
    inertial_sensor = new pros::Imu(GYRO_PORT);

    // =============================================================================
    // INITIALIZE PID CONTROLLERS
    // =============================================================================
    
    printf("Creating PID controller objects...\n");
    
    // Linear controller (for driving to points) - PROVEN WORKING
    linear_controller = new lemlib::ControllerSettings(
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
    angular_controller = new lemlib::ControllerSettings(
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
    angular_turn_controller = new lemlib::ControllerSettings(
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
    angular_short_turn_controller = new lemlib::ControllerSettings(
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
    // INITIALIZE DRIVE CURVES
    // =============================================================================
    
    printf("Creating drive curve objects...\n");
    
    // Drive curves for smooth joystick control - EXACT match to working code
    throttleCurve = new lemlib::ExpoDriveCurve(3,      // joystick deadband out of 127
                                             10,     // minimum output where drivetrain will move out of 127
                                             1.019); // expo curve gain

    steerCurve = new lemlib::ExpoDriveCurve(3,      // joystick deadband out of 127
                                          10,     // minimum output where drivetrain will move out of 127
                                          1.019); // expo curve gain

    // =============================================================================
    // INITIALIZE DRIVETRAIN
    // =============================================================================
    
    printf("Creating drivetrain objects...\n");
    
    // Drivetrain configuration - EXACTLY like working_code.txt
    drivetrain = new lemlib::Drivetrain(
        left_motor_group,
        right_motor_group,
        DRIVE_TRACK_WIDTH,              // 12.5 inches - PROVEN WORKING
        lemlib::Omniwheel::NEW_325,     // EXACT match to working code
        DRIVE_RPM,                      // 450 RPM - blue cartridge motors
        2 // horizontalDrift - 2 for omni wheels (exact match to working code)
    );

    // Create the lemlib_drivetrain pointer for compatibility
    lemlib_drivetrain = drivetrain;

    // =============================================================================
    // INITIALIZE ODOMETRY SENSORS
    // =============================================================================
    
    printf("Creating odometry sensors object...\n");
    
    odometry_sensors = new lemlib::OdomSensors(
        vertical_tracking_wheel,   // vertical1 - ENABLED (tracking wheel works!)
        nullptr,                   // vertical2
        horizontal_tracking_wheel, // horizontal1 - ENABLED (tracking wheel works!)
        nullptr,                   // horizontal2
        inertial_sensor           // imu - for heading
    );

    // =============================================================================
    // INITIALIZE CHASSIS OBJECTS
    // =============================================================================
    
    printf("Creating chassis objects...\n");
    
    // LemLib chassis object - EXACT match to working code including drive curves
    chassis = new lemlib::Chassis(*drivetrain,
                                 *linear_controller,
                                 *angular_controller,
                                 *odometry_sensors,
                                 throttleCurve,
                                 steerCurve);

    // Additional chassis objects for different turn scenarios (like working_code.txt)
    chassis_turn = new lemlib::Chassis(*drivetrain,
                                      *linear_controller, 
                                      *angular_turn_controller,
                                      *odometry_sensors,
                                      throttleCurve,
                                      steerCurve);

    chassis_turn_short = new lemlib::Chassis(*drivetrain,
                                            *linear_controller,
                                            *angular_short_turn_controller, 
                                            *odometry_sensors,
                                            throttleCurve,
                                            steerCurve);

    // =============================================================================
    // CALIBRATE CHASSIS
    // =============================================================================
    
    printf("Calibrating chassis...\n");
    
    // Calibrate the chassis (this will calibrate IMU and start odometry)
    chassis->calibrate();
    
    // Wait for calibration to complete
    printf("Waiting for IMU calibration...\n");
    pros::delay(2000);  // Give IMU time to fully calibrate
    
    // Initialize odometry by setting a known pose
    printf("Setting initial pose to (0, 0, 0)...\n");
    chassis->setPose(0, 0, 0);
    pros::delay(100);
    
    // Verify odometry is working
    lemlib::Pose current_pose = chassis->getPose();
    printf("Initial pose set to: (%.2f, %.2f, %.2f°)\n", 
           current_pose.x, current_pose.y, current_pose.theta);
    
    printf("LemLib initialization complete!\n");
    
    // Mark as initialized to prevent duplicate calls
    lemlib_initialized = true;
    printf("✅ LemLib initialization flag set - future calls will be ignored\n");
}

bool isLemLibInitialized() {
    return lemlib_initialized;
}

bool validateLemLibInitialization() {
    printf("Validating LemLib initialization...\n");
    
    // Check if initialization was completed
    if (!lemlib_initialized) {
        printf("❌ ERROR: LemLib not initialized! Call initializeLemLib() first.\n");
        return false;
    }
    
    // Check motors
    if (!left_front_motor || !left_middle_motor || !left_back_motor ||
        !right_front_motor || !right_middle_motor || !right_back_motor) {
        printf("❌ ERROR: Motor objects not initialized!\n");
        return false;
    }
    
    // Check motor groups
    if (!left_motor_group || !right_motor_group) {
        printf("❌ ERROR: Motor group objects not initialized!\n");
        return false;
    }
    
    // Check tracking wheels
    if (!vertical_encoder || !horizontal_encoder || 
        !vertical_tracking_wheel || !horizontal_tracking_wheel) {
        printf("❌ ERROR: Tracking wheel objects not initialized!\n");
        return false;
    }
    
    // Check IMU
    if (!inertial_sensor) {
        printf("❌ ERROR: IMU object not initialized!\n");
        return false;
    }
    
    // Check PID controllers
    if (!linear_controller || !angular_controller || 
        !angular_turn_controller || !angular_short_turn_controller) {
        printf("❌ ERROR: PID controller objects not initialized!\n");
        return false;
    }
    
    // Check drive curves
    if (!throttleCurve || !steerCurve) {
        printf("❌ ERROR: Drive curve objects not initialized!\n");
        return false;
    }
    
    // Check drivetrain and chassis
    if (!drivetrain || !chassis || !chassis_turn || !chassis_turn_short) {
        printf("❌ ERROR: Chassis objects not initialized!\n");
        return false;
    }
    
    // Check odometry sensors
    if (!odometry_sensors) {
        printf("❌ ERROR: Odometry sensors object not initialized!\n");
        return false;
    }
    
    printf("✅ All LemLib objects successfully initialized!\n");
    return true;
}