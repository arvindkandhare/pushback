/**
 * \file autonomous.cpp
 *
 * Autonomous system implementation for VEX Push Back 2024-25.
 * Implements odometry-based navigation with AWP-focused routines.
 */

#include "autonomous.h"
#include "lemlib_config.h"
#include <utility>

// =============================================================================
// PID Controller Implementation
// =============================================================================

PIDController::PIDController(double kp, double ki, double kd) 
    : kp(kp), ki(ki), kd(kd), previous_error(0), integral(0), last_time(0) {}

double PIDController::calculate(double error) {
    uint32_t current_time = pros::millis();
    double dt = (current_time - last_time) / 1000.0; // Convert to seconds
    
    if (last_time == 0) dt = 0.02; // Default 20ms for first call
    
    // Integral term with windup protection
    integral += error * dt;
    if (integral > 50) integral = 50;
    if (integral < -50) integral = -50;
    
    // Derivative term
    double derivative = (dt > 0) ? (error - previous_error) / dt : 0;
    
    // Calculate PID output
    double output = kp * error + ki * integral + kd * derivative;
    
    // Update for next iteration
    previous_error = error;
    last_time = current_time;
    
    return output;
}

void PIDController::reset() {
    previous_error = 0;
    integral = 0;
    last_time = 0;
}

// =============================================================================
// Auto Selector Implementation
// =============================================================================

AutoSelector::AutoSelector() 
    : selected_mode(AutoMode::DISABLED), selector_position(0), mode_confirmed(false) {}

void AutoSelector::displayOptions() {
    const char* mode_names[] = {
        "DISABLED",
        "Red Left AWP",
        "Red Left BONUS", 
        "Red Right AWP",
        "Red Right BONUS",
        "Skills",
        "Test: Drive",
        "Test: Turn",
        "Test: Navigation",
        "Test: Odometry"
    };
    
    // Display on controller screen only
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (master.is_connected()) {
        if (mode_confirmed) {
            master.print(0, 0, "READY: %s", mode_names[selector_position]);
            master.print(1, 0, "A:change L1+L2:test");
        } else {
            master.print(0, 0, "%d: %s", selector_position, mode_names[selector_position]);
            master.print(1, 0, "UP/DN:change A:ok");
        }
    }
}

void AutoSelector::handleInput() {
    // Use controller instead of LCD buttons
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    // Read controller button states
    bool left_pressed = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);
    bool right_pressed = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);
    bool up_pressed = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP);
    bool down_pressed = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);
    bool a_pressed = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
    
    if (!mode_confirmed) {
        // Navigation mode
        if (left_pressed || down_pressed) {
            selector_position--;
            if (selector_position < 0) selector_position = 9;
            printf("Selected mode: %d\n", selector_position);
        }
        
        if (right_pressed || up_pressed) {
            selector_position++;
            if (selector_position > 9) selector_position = 0;
            printf("Selected mode: %d\n", selector_position);
        }
        
        if (a_pressed) {
            selected_mode = static_cast<AutoMode>(selector_position);
            mode_confirmed = true;
            printf("Autonomous mode CONFIRMED: %d\n", selector_position);
            
            // Print mode name for clarity
            const char* mode_names[] = {
                "DISABLED", "Red Left AWP (SOLO AWP)", "Red Left BONUS (AWP+Points)", 
                "Red Right AWP (Original)", "Red Right BONUS (AWP+Points)", "Skills",
                "Test: Drive", "Test: Turn", "Test: Navigation", "Test: Odometry"
            };
            printf("Mode: %s\n", mode_names[selector_position]);
        }
    } else {
        // Confirmation mode - allow changing selection
        if (a_pressed) {
            mode_confirmed = false;
            printf("Mode deselected - can change again\n");
        }
    }
}

AutoMode AutoSelector::getSelectedMode() {
    return selected_mode;
}

bool AutoSelector::isModeConfirmed() {
    return mode_confirmed;
}

void AutoSelector::update() {
    handleInput();
    displayOptions();
}

// =============================================================================
// Autonomous System Implementation  
// =============================================================================

AutonomousSystem::AutonomousSystem(Drivetrain* dt, PTO* pto, IndexerSystem* indexer)
    : vertical_encoder(VERTICAL_ENCODER_PORT, false),
      horizontal_encoder(HORIZONTAL_ENCODER_PORT, false),
      gyro(GYRO_PORT),
      drivetrain(dt),
      pto_system(pto),
      indexer_system(indexer),
      current_position(0, 0, 0),
      target_position(0, 0, 0),
      drive_pid(DRIVE_KP, DRIVE_KI, DRIVE_KD),
      turn_pid(TURN_KP, TURN_KI, TURN_KD),
      odometry_initialized(false),
      autonomous_running(false),
      last_update_time(0) {}

void AutonomousSystem::initialize() {
    printf("Initializing Autonomous System...\n");
    
    // Reset encoders
    vertical_encoder.reset();
    horizontal_encoder.reset();
    
    // Calibrate gyroscope
    printf("Calibrating gyro...\n");
    gyro.reset();
    
    // Wait for gyro calibration (up to 3 seconds)
    int calibration_time = 0;
    while (gyro.is_calibrating() && calibration_time < 3000) {
        pros::delay(10);
        calibration_time += 10;
    }
    
    if (gyro.is_calibrating()) {
        printf("WARNING: Gyro calibration timeout!\n");
    } else {
        printf("Gyro calibration complete\n");
    }
    
    // Initialize position
    setPosition(0, 0, 0);
    
    // Reset PID controllers
    drive_pid.reset();
    turn_pid.reset();
    
    odometry_initialized = true;
    printf("Autonomous System initialized\n");
    
    // LemLib is already initialized in initializeGlobalSubsystems()
    // No need to initialize again here
    
    pros::delay(500);
    printf("Autonomous System initialization complete\n");
}

void AutonomousSystem::updateOdometry() {
    if (!odometry_initialized) return;
    
    uint32_t current_time = pros::millis();
    if (last_update_time == 0) {
        last_update_time = current_time;
        return;
    }
    
    double dt = (current_time - last_update_time) / 1000.0;
    last_update_time = current_time;
    
    // Read encoder values
    double vertical_ticks = vertical_encoder.get_value();
    double horizontal_ticks = horizontal_encoder.get_value();
    
    // Convert to distance (adjust based on your encoder setup)
    double vertical_distance = (vertical_ticks * TRACKING_WHEEL_CIRCUMFERENCE) / 360.0;
    double horizontal_distance = (horizontal_ticks * TRACKING_WHEEL_CIRCUMFERENCE) / 360.0;
    
    // Get gyro heading
    double heading_rad = gyro.get_heading() * M_PI / 180.0;
    
    // Reset encoders for next iteration
    vertical_encoder.reset();
    horizontal_encoder.reset();
    
    // Update position using odometry equations
    double cos_h = cos(heading_rad);
    double sin_h = sin(heading_rad);
    
    current_position.x += vertical_distance * cos_h - horizontal_distance * sin_h;
    current_position.y += vertical_distance * sin_h + horizontal_distance * cos_h;
    current_position.heading = gyro.get_heading();
}

void AutonomousSystem::update() {
    updateOdometry();
    auto_selector.update();
}

void AutonomousSystem::setPosition(double x, double y, double heading) {
    current_position.x = x;
    current_position.y = y;
    current_position.heading = heading;
    
    // Reset gyro to match heading
    gyro.set_heading(heading);
    
    printf("Position set to: (%.2f, %.2f, %.2f°)\n", x, y, heading);
}

Position AutonomousSystem::getPosition() {
    return current_position;
}

double AutonomousSystem::normalizeAngle(double angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

double AutonomousSystem::getDistanceToTarget() {
    double dx = target_position.x - current_position.x;
    double dy = target_position.y - current_position.y;
    return sqrt(dx * dx + dy * dy);
}

double AutonomousSystem::getAngleToTarget() {
    double dx = target_position.x - current_position.x;
    double dy = target_position.y - current_position.y;
    return atan2(dy, dx) * 180.0 / M_PI;
}

double AutonomousSystem::getHeadingError(double target_heading) {
    return normalizeAngle(target_heading - current_position.heading);
}

void AutonomousSystem::driveToPoint(double x, double y, double max_speed, double timeout_ms) {
    target_position.x = x;
    target_position.y = y;
    
    // Use LemLib chassis to move to the point
    printf("Driving to point (LemLib): (%.2f, %.2f)\n", x, y);
    chassis.moveToPoint(x, y, timeout_ms);
    chassis.waitUntilDone();
    printf("Drive to point (LemLib) complete\n");
}

void AutonomousSystem::turnToHeading(double heading, double max_speed, double timeout_ms) {
    // Use LemLib to perform heading turn
    printf("Turning to heading (LemLib): %.2f°\n", heading);
    chassis.turnToHeading(heading, timeout_ms);
    chassis.waitUntilDone();
    printf("Turn to heading (LemLib) complete\n");
}

void AutonomousSystem::driveDistance(double distance, double heading, double max_speed, double timeout_ms) {
    // If no heading specified, maintain current heading
    if (heading == -999) {
        heading = current_position.heading;
    }
    
    // Calculate target position
    double target_x = current_position.x + distance * cos(heading * M_PI / 180.0);
    double target_y = current_position.y + distance * sin(heading * M_PI / 180.0);
    
    driveToPoint(target_x, target_y, max_speed, timeout_ms);
}

AutoSelector& AutonomousSystem::getSelector() {
    return auto_selector;
}

void AutonomousSystem::stopAllMovement() {
    drivetrain->stop();
    autonomous_running = false;
}

bool AutonomousSystem::isMovementComplete() {
    double distance_error = getDistanceToTarget();
    double heading_error = getHeadingError(target_position.heading);
    
    return (distance_error < POSITION_THRESHOLD && fabs(heading_error) < HEADING_THRESHOLD);
}

void AutonomousSystem::printPosition() {
    printf("Robot Position: (%.2f, %.2f, %.2f°)\n", 
           current_position.x, current_position.y, current_position.heading);
}

// =============================================================================
// Autonomous Route Implementations (Placeholders - to be developed)
// =============================================================================

void AutonomousSystem::executeRedLeftAWP() {
    printf("Executing Red Left AWP Route (Mirrored from proven Red Right route)\n");
    autonomous_running = true;

    // Set starting pose for LEFT side (mirror of Red Right's 60°)
    chassis.setPose(0, 0, 120);  // 120° = northwest direction (mirror of 60°)

    // START INTAKE
    indexer_system->startInput();

    // Move forward ~35.5" at mirrored angle (120° instead of 60°)
    chassis.moveToPoint(35.5 * sin(120 * M_PI / 180.0), 35.5 * cos(120 * M_PI / 180.0), 5000);
    chassis.waitUntilDone();
    
    pros::delay(100);
    
    // Turn to 180° (same as Red Right - facing toward red alliance)
    chassis.turnToHeading(180, 3000);
    chassis.waitUntilDone();
    
    pros::delay(100);

    // Back up ~12" (same positioning logic)
    auto pose = chassis.getPose();
    chassis.moveToPoint(pose.x - 12 * sin(180 * M_PI / 180.0), 
                       pose.y - 12 * cos(180 * M_PI / 180.0), 3000);
    chassis.waitUntilDone();

    // BACKSCORING MIDDLE - execute indexer back scoring sequence
    indexer_system->setMidGoalMode();
    indexer_system->executeBack();
    pros::delay(700); // brief pause for scoring
    indexer_system->stopAll();

    pros::delay(50);
    
    // Continue with mirrored navigation pattern
    pose = chassis.getPose();
    chassis.moveToPoint(pose.x + 27 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 27 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis.waitUntilDone();
    
    // Mirror of 160° → 200° (opposite side approach)
    chassis.turnToHeading(200, 3000);
    chassis.waitUntilDone();
    
    pros::delay(50);
    
    pose = chassis.getPose();
    chassis.moveToPoint(pose.x + 22 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 22 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis.waitUntilDone();
    
    pros::delay(50);
    
    // Mirror of 225° → 315° (approach match load from left side)
    chassis.turnToHeading(315, 3000);
    chassis.waitUntilDone();
    
    pose = chassis.getPose();
    chassis.moveToPoint(pose.x + 23.5 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 23.5 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis.waitUntilDone();

    pros::delay(1000);
    
    // START INTAKE FROM MATCH LOAD (left side)
    indexer_system->startInput();

    // Mirror of 231° → 309° (approach from left match load zone)
    chassis.turnToHeading(309, 3000);
    chassis.waitUntilDone();
    
    pose = chassis.getPose();
    chassis.moveToPoint(pose.x - 35 * sin(pose.theta * M_PI / 180.0),
                       pose.y - 35 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis.waitUntilDone();

    pros::delay(50);

    // TOP BACKSCORING - use back/top indexer (same as Red Right)
    indexer_system->setTopGoalMode();
    indexer_system->executeBack();
    pros::delay(1200);
    indexer_system->stopAll();

    printf("Red Left AWP finished!\n");

    autonomous_running = false;
    printf("Red Left AWP Route Complete\n");
}

void AutonomousSystem::executeRedLeftBonus() {
    printf("Executing Red Left BONUS Route (AWP + Maximum Points - Mirrored)\n");
    autonomous_running = true;

    // Set starting pose for LEFT side (mirror of Red Right's 60°)
    chassis.setPose(0, 0, 120);  // 120° = northwest direction

    // START INTAKE immediately for maximum block collection
    indexer_system->startInput();

    printf("BONUS Phase 1: Aggressive AWP completion\n");
    
    // Use the proven working path but mirrored for left side
    chassis.moveToPoint(35.5 * sin(120 * M_PI / 180.0), 35.5 * cos(120 * M_PI / 180.0), 4000); // Faster, mirrored
    chassis.waitUntilDone();
    
    // Quick turn and score
    chassis.turnToHeading(180, 2500); // Faster turn (same as Red Right)
    chassis.waitUntilDone();
    
    auto pose = chassis.getPose();
    chassis.moveToPoint(pose.x - 12 * sin(180 * M_PI / 180.0), 
                       pose.y - 12 * cos(180 * M_PI / 180.0), 2500); // Faster movement
    chassis.waitUntilDone();

    // FAST AWP SCORING
    indexer_system->setMidGoalMode();
    indexer_system->executeBack();
    pros::delay(600); // Shorter delay for speed
    indexer_system->stopAll();

    printf("BONUS Phase 2: Maximum point collection\n");
    
    // Continue with proven path but collect more blocks and mirrored angles
    indexer_system->startInput(); // Keep collecting throughout
    
    pose = chassis.getPose();
    chassis.moveToPoint(pose.x + 27 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 27 * cos(pose.theta * M_PI / 180.0), 2500);
    chassis.waitUntilDone();
    
    // Mirror of 160° → 200° (left side approach)
    chassis.turnToHeading(200, 2000); // Faster
    chassis.waitUntilDone();
    
    pose = chassis.getPose();
    chassis.moveToPoint(pose.x + 22 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 22 * cos(pose.theta * M_PI / 180.0), 2500);
    chassis.waitUntilDone();
    
    // BONUS: Additional scoring opportunity
    indexer_system->setMidGoalMode();
    indexer_system->executeBack();
    pros::delay(500); // Quick score
    indexer_system->stopAll();
    
    // Continue to match load zone faster (mirror of 225° → 315°)
    chassis.turnToHeading(315, 2000);
    chassis.waitUntilDone();
    
    pose = chassis.getPose();
    chassis.moveToPoint(pose.x + 23.5 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 23.5 * cos(pose.theta * M_PI / 180.0), 2500);
    chassis.waitUntilDone();

    // Aggressive intake from match load (left side)
    indexer_system->startInput();
    pros::delay(800); // Slightly longer to grab more blocks

    // Mirror of 231° → 309° (left match load approach)
    chassis.turnToHeading(309, 2000);
    chassis.waitUntilDone();
    
    pose = chassis.getPose();
    chassis.moveToPoint(pose.x - 35 * sin(pose.theta * M_PI / 180.0),
                       pose.y - 35 * cos(pose.theta * M_PI / 180.0), 2500);
    chassis.waitUntilDone();

    // FINAL HIGH-VALUE SCORING
    indexer_system->setTopGoalMode();
    indexer_system->executeBack();
    pros::delay(1000); // Ensure all blocks are scored
    indexer_system->stopAll();

    printf("Left BONUS Complete!\n");
    autonomous_running = false;
    printf("Red Left BONUS Route Complete - Maximum Points + AWP achieved\n");
}

void AutonomousSystem::executeRedRightAWP() {
    printf("Executing Red Right AWP Route (Original working route moved here)\n");
    autonomous_running = true;

    // Set starting pose for RIGHT side (this was the original working code)
    chassis.setPose(0, 0, 60);

    // START INTAKE
    indexer_system->startInput();

    // Move forward ~35.5" (original working movement)
    chassis.moveToPoint(35.5 * sin(60 * M_PI / 180.0), 35.5 * cos(60 * M_PI / 180.0), 5000);
    chassis.waitUntilDone();
    
    pros::delay(100);
    
    // Turn to 180°
    chassis.turnToHeading(180, 3000);
    chassis.waitUntilDone();
    
    pros::delay(100);

    // Back up ~12"
    auto pose = chassis.getPose();
    chassis.moveToPoint(pose.x - 12 * sin(180 * M_PI / 180.0), 
                       pose.y - 12 * cos(180 * M_PI / 180.0), 3000);
    chassis.waitUntilDone();

    // BACKSCORING MIDDLE - execute indexer back scoring sequence
    indexer_system->setMidGoalMode();
    indexer_system->executeBack();
    pros::delay(700); // brief pause for scoring
    indexer_system->stopAll();

    pros::delay(50);
    
    // Continue with remaining movements
    pose = chassis.getPose();
    chassis.moveToPoint(pose.x + 27 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 27 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis.waitUntilDone();
    
    chassis.turnToHeading(160, 3000);
    chassis.waitUntilDone();
    
    pros::delay(50);
    
    pose = chassis.getPose();
    chassis.moveToPoint(pose.x + 22 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 22 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis.waitUntilDone();
    
    pros::delay(50);
    
    chassis.turnToHeading(225, 3000);
    chassis.waitUntilDone();
    
    pose = chassis.getPose();
    chassis.moveToPoint(pose.x + 23.5 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 23.5 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis.waitUntilDone();

    pros::delay(1000);
    
    // START INTAKE FROM MATCH LOAD
    indexer_system->startInput();

    chassis.turnToHeading(231, 3000);
    chassis.waitUntilDone();
    
    pose = chassis.getPose();
    chassis.moveToPoint(pose.x - 35 * sin(pose.theta * M_PI / 180.0),
                       pose.y - 35 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis.waitUntilDone();

    pros::delay(50);

    // TOP BACKSCORING - use back/top indexer
    indexer_system->setTopGoalMode();
    indexer_system->executeBack();
    pros::delay(1200);
    indexer_system->stopAll();

    printf("Red Right AWP finished!\n");

    autonomous_running = false;
    printf("Red Right AWP Route Complete\n");
}

void AutonomousSystem::executeRedRightBonus() {
    printf("Executing Red Right BONUS Route (AWP + Maximum Points)\n");
    autonomous_running = true;

    // Set starting pose for RIGHT side
    chassis.setPose(0, 0, 60);

    // START INTAKE immediately for maximum block collection
    indexer_system->startInput();

    printf("BONUS Phase 1: Aggressive AWP completion\n");
    
    // Use the proven working path but optimize for speed and points
    chassis.moveToPoint(35.5 * sin(60 * M_PI / 180.0), 35.5 * cos(60 * M_PI / 180.0), 4000); // Faster
    chassis.waitUntilDone();
    
    // Quick turn and score
    chassis.turnToHeading(180, 2500); // Faster turn
    chassis.waitUntilDone();
    
    auto pose = chassis.getPose();
    chassis.moveToPoint(pose.x - 12 * sin(180 * M_PI / 180.0), 
                       pose.y - 12 * cos(180 * M_PI / 180.0), 2500); // Faster movement
    chassis.waitUntilDone();

    // FAST AWP SCORING
    indexer_system->setMidGoalMode();
    indexer_system->executeBack();
    pros::delay(600); // Shorter delay for speed
    indexer_system->stopAll();

    printf("BONUS Phase 2: Maximum point collection\n");
    
    // Continue with proven path but collect more blocks
    indexer_system->startInput(); // Keep collecting throughout
    
    pose = chassis.getPose();
    chassis.moveToPoint(pose.x + 27 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 27 * cos(pose.theta * M_PI / 180.0), 2500);
    chassis.waitUntilDone();
    
    chassis.turnToHeading(160, 2000); // Faster
    chassis.waitUntilDone();
    
    pose = chassis.getPose();
    chassis.moveToPoint(pose.x + 22 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 22 * cos(pose.theta * M_PI / 180.0), 2500);
    chassis.waitUntilDone();
    
    // BONUS: Additional scoring opportunity
    indexer_system->setMidGoalMode();
    indexer_system->executeBack();
    pros::delay(500); // Quick score
    indexer_system->stopAll();
    
    // Continue to match load zone faster
    chassis.turnToHeading(225, 2000);
    chassis.waitUntilDone();
    
    pose = chassis.getPose();
    chassis.moveToPoint(pose.x + 23.5 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 23.5 * cos(pose.theta * M_PI / 180.0), 2500);
    chassis.waitUntilDone();

    // Aggressive intake from match load
    indexer_system->startInput();
    pros::delay(800); // Slightly longer to grab more blocks

    chassis.turnToHeading(231, 2000);
    chassis.waitUntilDone();
    
    pose = chassis.getPose();
    chassis.moveToPoint(pose.x - 35 * sin(pose.theta * M_PI / 180.0),
                       pose.y - 35 * cos(pose.theta * M_PI / 180.0), 2500);
    chassis.waitUntilDone();

    // FINAL HIGH-VALUE SCORING
    indexer_system->setTopGoalMode();
    indexer_system->executeBack();
    pros::delay(1000); // Ensure all blocks are scored
    indexer_system->stopAll();

    printf("BONUS Route Complete!\n");
    autonomous_running = false;
    printf("Red Right BONUS Route Complete - Maximum Points + AWP achieved\n");
}

void AutonomousSystem::executeSkillsRoutine() {
    printf("Executing Skills Routine - Running Red Right AWP for testing\n");
    autonomous_running = true;
    
    // For testing purposes, run the Red Right AWP route in Skills mode
    executeRedRightAWP();
    
    autonomous_running = false;
    printf("Skills Routine Complete\n");
}

void AutonomousSystem::runAutonomous() {
    if (!odometry_initialized) {
        printf("ERROR: Autonomous system not initialized!\n");
        return;
    }
    
    AutoMode mode = auto_selector.getSelectedMode();
    printf("Running autonomous mode: %d\n", static_cast<int>(mode));
    
    switch (mode) {
        case AutoMode::RED_LEFT_AWP:
            executeRedLeftAWP();
            break;
            
        case AutoMode::RED_LEFT_BONUS:
            executeRedLeftBonus();
            break;
            
        case AutoMode::RED_RIGHT_AWP:
            executeRedRightAWP();
            break;
            
        case AutoMode::RED_RIGHT_BONUS:
            executeRedRightBonus();
            break;
            
        case AutoMode::SKILLS:
            executeSkillsRoutine();
            break;
            
        case AutoMode::TEST_DRIVE:
            printf("🚗 TEST: Drive 24 inches forward\n");
            printf("Current PID gains: P=%.2f, I=%.3f, D=%.2f\n", DRIVE_KP, DRIVE_KI, DRIVE_KD);
            
            // First try a basic motor test
            printf("=== BASIC MOTOR TEST ===\n");
            printf("Testing raw motor movement...\n");
            
            // Test individual motors from LemLib config
            printf("Moving left motors forward...\n");
            left_motor_group.move_velocity(100);  // 100 RPM forward
            pros::delay(1000);  // 1 second
            left_motor_group.brake();
            
            printf("Moving right motors forward...\n");
            right_motor_group.move_velocity(100);  // 100 RPM forward
            pros::delay(1000);  // 1 second
            right_motor_group.brake();
            
            printf("Moving both sides forward...\n");
            left_motor_group.move_velocity(100);
            right_motor_group.move_velocity(100);
            pros::delay(1000);  // 1 second
            left_motor_group.brake();
            right_motor_group.brake();
            
            printf("Basic motor test complete. Now testing LemLib...\n");
            testStraightDrive(24.0);  // Test 24" drive
            break;
            
        case AutoMode::TEST_TURN:
            printf("🔄 TEST: Turn 90 degrees\n");
            printf("Current PID gains: P=%.2f, I=%.3f, D=%.2f\n", TURN_KP, TURN_KI, TURN_KD);
            testTurnAccuracy(90.0);   // Test 90° turn
            break;
            
        case AutoMode::TEST_NAVIGATION:
            testPointToPoint();       // Test navigation accuracy
            break;
            
        case AutoMode::TEST_ODOMETRY:
            testOdometryAccuracy();   // Test odometry with manual verification
            break;
            
        case AutoMode::DISABLED:
        default:
            printf("Autonomous disabled or invalid mode\n");
            break;
    }
}

// =============================================================================
// Testing and Calibration Functions
// =============================================================================

void AutonomousSystem::testStraightDrive(double distance) {
    printf("=== STRAIGHT DRIVE TEST ===\n");
    printf("Target distance: %.2f inches\n", distance);
    
    // Record starting position
    Position start_pos = current_position;
    printf("Starting position: (%.2f, %.2f, %.2f°)\n", 
           start_pos.x, start_pos.y, start_pos.heading);
    
    // Reset position for clean test
    setPosition(0, 0, 0);
    
    // Drive straight
    uint32_t start_time = pros::millis();
    driveDistance(distance, 0);  // Drive forward, maintain 0° heading
    uint32_t end_time = pros::millis();
    
    // Check results
    Position final_pos = current_position;
    double actual_distance = sqrt(final_pos.x * final_pos.x + final_pos.y * final_pos.y);
    double distance_error = actual_distance - distance;
    double heading_error = final_pos.heading;
    
    printf("=== RESULTS ===\n");
    printf("Target: %.2f inches\n", distance);
    printf("Actual: %.2f inches\n", actual_distance);
    printf("Error: %.2f inches (%.1f%%)\n", distance_error, (distance_error/distance)*100);
    printf("Heading drift: %.2f degrees\n", heading_error);
    printf("Time taken: %d ms\n", end_time - start_time);
    
    if (fabs(distance_error) < 1.0 && fabs(heading_error) < 3.0) {
        printf("✅ PASS: Drive accuracy acceptable\n");
        printf("   PID tuning looks good!\n");
    } else {
        printf("❌ FAIL: Needs calibration\n");
        if (fabs(distance_error) >= 1.0) {
            printf("   📏 Distance error too large (%.2f\")\n", distance_error);
            if (distance_error > 0) {
                printf("   Robot overshoots - try reducing DRIVE_KP to %.2f\n", DRIVE_KP - 0.2);
            } else {
                printf("   Robot undershoots - try increasing DRIVE_KP to %.2f\n", DRIVE_KP + 0.2);
            }
            printf("   Or adjust TRACKING_WHEEL_DIAMETER in config.h\n");
        }
        if (fabs(heading_error) >= 3.0) {
            printf("   🧭 Heading drift too large (%.2f°)\n", heading_error);
            printf("   Check wheel alignment or adjust turn PID\n");
            if (fabs(heading_error) > 10.0) {
                printf("   Try reducing TURN_KP to %.2f\n", TURN_KP - 0.2);
            }
        }
    }
}

void AutonomousSystem::testTurnAccuracy(double angle) {
    printf("=== TURN ACCURACY TEST ===\n");
    printf("Target angle: %.2f degrees\n", angle);
    
    // Reset position
    setPosition(0, 0, 0);
    
    // Perform turn
    uint32_t start_time = pros::millis();
    turnToHeading(angle);
    uint32_t end_time = pros::millis();
    
    // Check results
    Position final_pos = current_position;
    double angle_error = final_pos.heading - angle;
    
    printf("=== RESULTS ===\n");
    printf("Target: %.2f degrees\n", angle);
    printf("Actual: %.2f degrees\n", final_pos.heading);
    printf("Error: %.2f degrees\n", angle_error);
    printf("Time taken: %d ms\n", end_time - start_time);
    
    if (fabs(angle_error) < 2.0) {
        printf("✅ PASS: Turn accuracy acceptable\n");
        printf("   PID tuning looks good!\n");
    } else {
        printf("❌ FAIL: Needs PID tuning\n");
        printf("   🔄 Turn error: %.2f degrees\n", angle_error);
        
        if (fabs(angle_error) > 10.0) {
            printf("   Large overshoot - try reducing TURN_KP to %.2f\n", TURN_KP - 0.3);
        } else if (fabs(angle_error) > 5.0) {
            printf("   Moderate error - try adjusting TURN_KP to %.2f\n", 
                   angle_error > 0 ? TURN_KP - 0.2 : TURN_KP + 0.2);
        } else {
            printf("   Small error - try increasing TURN_KI to %.3f\n", TURN_KI + 0.01);
        }
        
        if (end_time - start_time > 3000) {
            printf("   Slow response - try increasing TURN_KP to %.2f\n", TURN_KP + 0.2);
        }
    }
}

void AutonomousSystem::testPointToPoint() {
    printf("=== POINT-TO-POINT NAVIGATION TEST ===\n");
    
    // Define test path (square pattern)
    Position waypoints[] = {
        {0, 0, 0},      // Start
        {24, 0, 0},     // Point 1: 24" forward
        {24, 24, 0},    // Point 2: 24" right
        {0, 24, 0},     // Point 3: 24" back
        {0, 0, 0}       // Return to start
    };
    
    setPosition(0, 0, 0);
    
    uint32_t total_start_time = pros::millis();
    
    for (int i = 1; i < 5; i++) {
        printf("Moving to waypoint %d: (%.0f, %.0f)\n", 
               i, waypoints[i].x, waypoints[i].y);
        
        driveToPoint(waypoints[i].x, waypoints[i].y);
        
        Position current = current_position;
        double error = sqrt(pow(current.x - waypoints[i].x, 2) + 
                           pow(current.y - waypoints[i].y, 2));
        
        printf("Reached: (%.2f, %.2f) - Error: %.2f inches\n", 
               current.x, current.y, error);
        
        pros::delay(1000);  // Pause between waypoints
    }
    
    uint32_t total_end_time = pros::millis();
    
    // Final accuracy check
    Position final = current_position;
    double return_error = sqrt(final.x * final.x + final.y * final.y);
    
    printf("=== RESULTS ===\n");
    printf("Return to start error: %.2f inches\n", return_error);
    printf("Total test time: %d ms\n", total_end_time - total_start_time);
    
    if (return_error < 3.0) {
        printf("✅ PASS: Navigation system working well\n");
    } else {
        printf("❌ FAIL: Significant odometry drift\n");
        printf("   Check encoder mounting and wheel contact\n");
        printf("   Verify TRACKING_WHEEL_DIAMETER setting\n");
    }
}

void AutonomousSystem::testOdometryAccuracy() {
    printf("=== ODOMETRY ACCURACY TEST ===\n");
    printf("This test requires manual measurement!\n");
    printf("1. Mark robot's current position\n");
    printf("2. Press any controller button to continue...\n");
    
    // Wait for controller input
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    while (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
           !controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) &&
           !controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
           !controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
        pros::delay(20);
    }
    
    setPosition(0, 0, 0);
    printf("Position reset to (0, 0, 0°)\n");
    
    // Move in a complex pattern
    driveToPoint(18, 0);     // Forward 18"
    driveToPoint(18, 12);    // Right 12"
    driveToPoint(6, 12);     // Back 12"
    driveToPoint(6, 6);      // Back 6"
    driveToPoint(0, 6);      // Left 6"
    driveToPoint(0, 0);      // Return to start
    
    Position final = current_position;
    printf("=== FINAL POSITION ===\n");
    printf("Odometry says: (%.2f, %.2f, %.2f°)\n", 
           final.x, final.y, final.heading);
    printf("Manually measure distance from starting mark.\n");
    printf("Good accuracy: < 2 inches from start\n");
}

void AutonomousSystem::printSensorReadings() {
    printf("=== SENSOR READINGS ===\n");
    printf("Vertical encoder: %d ticks\n", vertical_encoder.get_value());
    printf("Horizontal encoder: %d ticks\n", horizontal_encoder.get_value());
    printf("Gyro heading: %.2f degrees\n", gyro.get_heading());
    printf("Gyro status: %s\n", gyro.is_calibrating() ? "Calibrating" : "Ready");
    printf("Position: (%.2f, %.2f, %.2f°)\n", 
           current_position.x, current_position.y, current_position.heading);
}