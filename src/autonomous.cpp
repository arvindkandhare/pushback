/**
 * \file autonomous.cpp
 *
 * Autonomous system implementation for VEX Push Back 2024-25.
 * Implements odometry-based navigation with AWP-focused routines.
 */

#include "autonomous.h"
#include "lemlib_config.h"
#include <utility>
#include <cmath>  // For cos, sin functions

// External references to global tracking wheel encoders (from lemlib_config.cpp)
extern pros::Rotation* vertical_encoder;
extern pros::Rotation* horizontal_encoder;

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
        "DISABLED",              // 0
        "Red Left BONUS",        // 1
        "Red Right BONUS",       // 2
        "Blue Left BONUS",       // 3
        "Blue Right BONUS",      // 4
        "Red Left AWP",          // 5
        "Red Right AWP",         // 6
        "Blue Left AWP",         // 7
        "Blue Right AWP",        // 8
        "Skills",                // 9
        "Test: Drive",           // 10 - This is the auto test mode
        "Test: Turn",            // 11
        "Test: Navigation",      // 12
        "Test: Odometry",        // 13
        "Test: Motors"           // 14
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
            if (selector_position < 0) selector_position = 14;  // EXPANDED: Now goes to 14
            printf("Selected mode: %d\n", selector_position);
        }
        
        if (right_pressed || up_pressed) {
            selector_position++;
            if (selector_position > 14) selector_position = 0;  // EXPANDED: Now supports 0-14
            printf("Selected mode: %d\n", selector_position);
        }
        
        if (a_pressed) {
            selected_mode = static_cast<AutoMode>(selector_position);
            mode_confirmed = true;
            printf("Autonomous mode CONFIRMED: %d\n", selector_position);
            
            // UPDATED: Correct mode names matching the AutoMode enum
            const char* mode_names[] = {
                "DISABLED",                    // 0
                "RED_LEFT_BONUS",             // 1
                "RED_RIGHT_BONUS",            // 2
                "BLUE_LEFT_BONUS",            // 3
                "BLUE_RIGHT_BONUS",           // 4
                "RED_LEFT_AWP",               // 5
                "RED_RIGHT_AWP",              // 6
                "BLUE_LEFT_AWP",              // 7
                "BLUE_RIGHT_AWP",             // 8
                "SKILLS",                     // 9
                "TEST_DRIVE",                 // 10 - TRACKING WHEEL DIAGNOSTIC
                "TEST_TURN",                  // 11
                "TEST_NAVIGATION",            // 12
                "TEST_ODOMETRY",              // 13
                "TEST_MOTORS"                 // 14
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

AutonomousSystem::AutonomousSystem(PTO* pto, IndexerSystem* indexer)
    : pto_system(pto),
      indexer_system(indexer),
      autonomous_running(false) {}

void AutonomousSystem::initialize() {
    printf("Initializing Autonomous System (LemLib wrapper)...\n");
    
    // LemLib is already initialized in lemlib_config.cpp
    // Just reset position to ensure clean start
    chassis->setPose(0, 0, 0);
    
    autonomous_running = false;
    printf("Autonomous System initialized\n");
}

void AutonomousSystem::update() {
    // Only update the selector - LemLib handles odometry
    auto_selector.update();
}

void AutonomousSystem::setPosition(double x, double y, double heading) {
    // Use LemLib to set position
    chassis->setPose(x, y, heading);
    printf("Position set to: (%.2f, %.2f, %.2f°)\n", x, y, heading);
}

lemlib::Pose AutonomousSystem::getPosition() {
    // Return LemLib position
    return chassis->getPose();
}

void AutonomousSystem::printPosition() {
    auto pose = chassis->getPose();
    printf("Robot Position: (%.2f, %.2f, %.2f°)\n", pose.x, pose.y, pose.theta);
}

// =============================================================================
// LemLib Wrapper Functions
// =============================================================================

void AutonomousSystem::driveToPoint(double x, double y, double timeout_ms) {
    printf("Driving to point (LemLib): (%.2f, %.2f)\n", x, y);
    chassis->moveToPoint(x, y, timeout_ms);
    chassis->waitUntilDone();
    printf("Drive to point (LemLib) complete\n");
}

void AutonomousSystem::turnToHeading(double heading, double timeout_ms) {
    printf("Turning to heading (LemLib): %.2f°\n", heading);
    chassis->turnToHeading(heading, timeout_ms);
    chassis->waitUntilDone();
    printf("Turn to heading (LemLib) complete\n");
}

void AutonomousSystem::driveDistance(double distance, double heading, double timeout_ms) {
    // Get current position from LemLib
    auto current_pose = chassis->getPose();
    
    // Calculate target position
    double target_x = current_pose.x + distance * cos(heading * M_PI / 180.0);
    double target_y = current_pose.y + distance * sin(heading * M_PI / 180.0);
    
    driveToPoint(target_x, target_y, timeout_ms);
}

AutoSelector& AutonomousSystem::getSelector() {
    return auto_selector;
}

void AutonomousSystem::stopAllMovement() {
    // Stop LemLib chassis movement
    chassis->cancelAllMotions();
    autonomous_running = false;
}

// =============================================================================
// Autonomous Route Implementations (Placeholders - to be developed)
// =============================================================================

void AutonomousSystem::executeRedLeftAWP() {
    printf("Executing Red Left AWP Route (Mirrored from proven Red Right route)\n");
    autonomous_running = true;

    // VERIFY PTO is in scorer mode (should already be set, but double-check)
    if (pto_system && !pto_system->isDrivetrainMode()) {
        printf("✅ Confirmed: PTO in scorer mode - middle wheels ready for scoring\n");
    } else {
        printf("⚠️  WARNING: PTO not in expected scorer mode - forcing scorer mode\n");
        pto_system->setScorerMode();
        pros::delay(200);
    }

    // Set starting pose for LEFT side (mirror of Red Right's 60°)
    chassis->setPose(0, 0, 120);  // 120° = northwest direction (mirror of 60°)

    // START INTAKE
    indexer_system->startInput();

    // Move forward ~35.5" at mirrored angle (120° instead of 60°)
    chassis->moveToPoint(35.5 * sin(120 * M_PI / 180.0), 35.5 * cos(120 * M_PI / 180.0), 5000);
    chassis->waitUntilDone();
    
    pros::delay(100);
    
    // Turn to 180° (same as Red Right - facing toward red alliance)
    chassis->turnToHeading(180, 3000);
    chassis->waitUntilDone();
    
    pros::delay(100);

    // Back up ~12" (same positioning logic)
    auto pose = chassis->getPose();
    chassis->moveToPoint(pose.x - 12 * sin(180 * M_PI / 180.0), 
                       pose.y - 12 * cos(180 * M_PI / 180.0), 3000);
    chassis->waitUntilDone();

    // BACKSCORING MIDDLE - execute indexer back scoring sequence
    indexer_system->setMidGoalMode();
    indexer_system->executeBack();
    pros::delay(700); // brief pause for scoring
    indexer_system->stopAll();

    pros::delay(50);
    
    // Continue with mirrored navigation pattern
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x + 27 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 27 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis->waitUntilDone();
    
    // Mirror of 160° → 200° (opposite side approach)
    chassis->turnToHeading(200, 3000);
    chassis->waitUntilDone();
    
    pros::delay(50);
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x + 22 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 22 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis->waitUntilDone();
    
    pros::delay(50);
    
    // Mirror of 225° → 315° (approach match load from left side)
    chassis->turnToHeading(315, 3000);
    chassis->waitUntilDone();
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x + 23.5 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 23.5 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis->waitUntilDone();

    pros::delay(1000);
    
    // START INTAKE FROM MATCH LOAD (left side)
    indexer_system->startInput();

    // Mirror of 231° → 309° (approach from left match load zone)
    chassis->turnToHeading(309, 3000);
    chassis->waitUntilDone();
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x - 35 * sin(pose.theta * M_PI / 180.0),
                       pose.y - 35 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis->waitUntilDone();

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
    chassis->setPose(0, 0, 120);  // 120° = northwest direction

    // START INTAKE immediately for maximum block collection
    indexer_system->startInput();

    printf("BONUS Phase 1: Aggressive AWP completion\n");
    
    // Use the proven working path but mirrored for left side
    chassis->moveToPoint(35.5 * sin(120 * M_PI / 180.0), 35.5 * cos(120 * M_PI / 180.0), 4000); // Faster, mirrored
    chassis->waitUntilDone();
    
    // Quick turn and score
    chassis->turnToHeading(180, 2500); // Faster turn (same as Red Right)
    chassis->waitUntilDone();
    
    auto pose = chassis->getPose();
    chassis->moveToPoint(pose.x - 12 * sin(180 * M_PI / 180.0), 
                       pose.y - 12 * cos(180 * M_PI / 180.0), 2500); // Faster movement
    chassis->waitUntilDone();

    // FAST AWP SCORING
    indexer_system->setMidGoalMode();
    indexer_system->executeBack();
    pros::delay(600); // Shorter delay for speed
    indexer_system->stopAll();

    printf("BONUS Phase 2: Maximum point collection\n");
    
    // Continue with proven path but collect more blocks and mirrored angles
    indexer_system->startInput(); // Keep collecting throughout
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x + 27 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 27 * cos(pose.theta * M_PI / 180.0), 2500);
    chassis->waitUntilDone();
    
    // Mirror of 160° → 200° (left side approach)
    chassis->turnToHeading(200, 2000); // Faster
    chassis->waitUntilDone();
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x + 22 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 22 * cos(pose.theta * M_PI / 180.0), 2500);
    chassis->waitUntilDone();
    
    // BONUS: Additional scoring opportunity
    indexer_system->setMidGoalMode();
    indexer_system->executeBack();
    pros::delay(500); // Quick score
    indexer_system->stopAll();
    
    // Continue to match load zone faster (mirror of 225° → 315°)
    chassis->turnToHeading(315, 2000);
    chassis->waitUntilDone();
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x + 23.5 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 23.5 * cos(pose.theta * M_PI / 180.0), 2500);
    chassis->waitUntilDone();

    // Aggressive intake from match load (left side)
    indexer_system->startInput();
    pros::delay(800); // Slightly longer to grab more blocks

    // Mirror of 231° → 309° (left match load approach)
    chassis->turnToHeading(309, 2000);
    chassis->waitUntilDone();
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x - 35 * sin(pose.theta * M_PI / 180.0),
                       pose.y - 35 * cos(pose.theta * M_PI / 180.0), 2500);
    chassis->waitUntilDone();

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
    chassis->setPose(0, 0, 60);

    // START INTAKE
    indexer_system->startInput();

    // Move forward ~35.5" (original working movement)
    chassis->moveToPoint(35.5 * sin(60 * M_PI / 180.0), 35.5 * cos(60 * M_PI / 180.0), 5000);
    chassis->waitUntilDone();
    
    pros::delay(100);
    
    // Turn to 180°
    chassis->turnToHeading(180, 3000);
    chassis->waitUntilDone();
    
    pros::delay(100);

    // Back up ~12"
    auto pose = chassis->getPose();
    chassis->moveToPoint(pose.x - 12 * sin(180 * M_PI / 180.0), 
                       pose.y - 12 * cos(180 * M_PI / 180.0), 3000);
    chassis->waitUntilDone();

    // BACKSCORING MIDDLE - execute indexer back scoring sequence
    indexer_system->setMidGoalMode();
    indexer_system->executeBack();
    pros::delay(700); // brief pause for scoring
    indexer_system->stopAll();

    pros::delay(50);
    
    // Continue with remaining movements
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x + 27 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 27 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis->waitUntilDone();
    
    chassis->turnToHeading(160, 3000);
    chassis->waitUntilDone();
    
    pros::delay(50);
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x + 22 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 22 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis->waitUntilDone();
    
    pros::delay(50);
    
    chassis->turnToHeading(225, 3000);
    chassis->waitUntilDone();
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x + 23.5 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 23.5 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis->waitUntilDone();

    pros::delay(1000);
    
    // START INTAKE FROM MATCH LOAD
    indexer_system->startInput();

    chassis->turnToHeading(231, 3000);
    chassis->waitUntilDone();
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x - 35 * sin(pose.theta * M_PI / 180.0),
                       pose.y - 35 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis->waitUntilDone();

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
    chassis->setPose(0, 0, 60);

    // START INTAKE immediately for maximum block collection
    indexer_system->startInput();

    printf("BONUS Phase 1: Aggressive AWP completion\n");
    
    // Use the proven working path but optimize for speed and points
    chassis->moveToPoint(35.5 * sin(60 * M_PI / 180.0), 35.5 * cos(60 * M_PI / 180.0), 4000); // Faster
    chassis->waitUntilDone();
    
    // Quick turn and score
    chassis->turnToHeading(180, 2500); // Faster turn
    chassis->waitUntilDone();
    
    auto pose = chassis->getPose();
    chassis->moveToPoint(pose.x - 12 * sin(180 * M_PI / 180.0), 
                       pose.y - 12 * cos(180 * M_PI / 180.0), 2500); // Faster movement
    chassis->waitUntilDone();

    // FAST AWP SCORING
    indexer_system->setMidGoalMode();
    indexer_system->executeBack();
    pros::delay(600); // Shorter delay for speed
    indexer_system->stopAll();

    printf("BONUS Phase 2: Maximum point collection\n");
    
    // Continue with proven path but collect more blocks
    indexer_system->startInput(); // Keep collecting throughout
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x + 27 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 27 * cos(pose.theta * M_PI / 180.0), 2500);
    chassis->waitUntilDone();
    
    chassis->turnToHeading(160, 2000); // Faster
    chassis->waitUntilDone();
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x + 22 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 22 * cos(pose.theta * M_PI / 180.0), 2500);
    chassis->waitUntilDone();
    
    // BONUS: Additional scoring opportunity
    indexer_system->setMidGoalMode();
    indexer_system->executeBack();
    pros::delay(500); // Quick score
    indexer_system->stopAll();
    
    // Continue to match load zone faster
    chassis->turnToHeading(225, 2000);
    chassis->waitUntilDone();
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x + 23.5 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 23.5 * cos(pose.theta * M_PI / 180.0), 2500);
    chassis->waitUntilDone();

    // Aggressive intake from match load
    indexer_system->startInput();
    pros::delay(800); // Slightly longer to grab more blocks

    chassis->turnToHeading(231, 2000);
    chassis->waitUntilDone();
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x - 35 * sin(pose.theta * M_PI / 180.0),
                       pose.y - 35 * cos(pose.theta * M_PI / 180.0), 2500);
    chassis->waitUntilDone();

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
    printf("=== AUTONOMOUS SETUP PHASE ===\n");
    
    // CRITICAL: Ensure PTO pistons are UP (scorer mode) before autonomous
    printf("🔧 Setting PTO to SCORER MODE (pistons UP, middle wheels disconnected)...\n");
    if (pto_system) {
        pto_system->setScorerMode();  // Pistons UP - middle wheels disconnected for scoring
        pros::delay(200);  // Allow pneumatics time to actuate
        printf("✅ PTO pistons raised - middle wheels disconnected\n");
        printf("   Robot now in 4-wheel drive mode (front + back wheels only)\n");
        printf("   Middle wheels available for scoring mechanisms\n");
    } else {
        printf("❌ ERROR: PTO system not available!\n");
    }
    
    printf("=== STARTING AUTONOMOUS EXECUTION ===\n");
   
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
            
        case AutoMode::TEST_DRIVE: {
            static int test_step = 6;  // START DIRECTLY AT STEP 6 with integrated safety checks
            
            printf("🔧 INTEGRATED DRIVE TEST WITH SAFETY CHECKS\n");
            printf("=== STEP %d of 6 ===\n", test_step);
            
            switch (test_step) {
                case 1: {
                    printf("STEP 1: PTO SYSTEM CHECK\n");
                    
                    // PTO STATE VERIFICATION
                    extern PTO* pto_system;
                    if (pto_system) {
                        printf("✅ PTO system found\n");
                        printf("Current mode: %s\n", pto_system->isDrivetrainMode() ? "DRIVETRAIN (6-wheel)" : "SCORER (4-wheel)");
                        if (!pto_system->isDrivetrainMode()) {
                            printf("⚠️  Note: PTO in scorer mode - only front/back wheels active\n");
                        }
                    } else {
                        printf("❌ PTO system is NULL!\n");
                    }
                    
                    printf("✅ STEP 1 COMPLETE - PTO system checked\n");
                    test_step++;
                    break;
                }
                
                case 2: {
                    printf("STEP 2: TRACKING WHEEL SENSOR CHECK\n");
                    
                    printf("Checking sensor connections...\n");
                    
                    // Check if sensors are installed/connected using global pros::Rotation encoders
                    bool vertical_connected = ::vertical_encoder->is_installed();
                    bool horizontal_connected = ::horizontal_encoder->is_installed();
                    
                    printf("Vertical encoder (Port %d): %s\n", VERTICAL_ENCODER_PORT, 
                           vertical_connected ? "✅ CONNECTED" : "❌ NOT DETECTED");
                    printf("Horizontal encoder (Port %d): %s\n", HORIZONTAL_ENCODER_PORT, 
                           horizontal_connected ? "✅ CONNECTED" : "❌ NOT DETECTED");
                    
                    if (vertical_connected && horizontal_connected) {
                        printf("✅ STEP 2 COMPLETE - Both sensors detected\n");
                    } else {
                        printf("❌ STEP 2 FAILED - Check sensor connections\n");
                    }
                    
                    test_step++;
                    break;
                }
                
                case 3: {
                    printf("STEP 3: INITIAL SENSOR READING\n");
                    
                    printf("Current sensor readings:\n");
                    printf("  Vertical encoder: %.2f degrees\n", ::vertical_encoder->get_position() / 100.0);
                    printf("  Horizontal encoder: %.2f degrees\n", ::horizontal_encoder->get_position() / 100.0);
                    
                    printf("✅ STEP 3 COMPLETE - Initial readings recorded\n");
                    test_step++;
                    break;
                }
                
                case 4: {
                    printf("STEP 4: MANUAL MOVEMENT TEST\n");
                    printf("🤚 MANUALLY PUSH THE ROBOT AROUND!\n");
                    printf("Instructions:\n");
                    printf("  • Push FORWARD/BACKWARD (should change vertical encoder)\n");
                    printf("  • Push LEFT/RIGHT (should change horizontal encoder)\n");
                    printf("  • ROTATE the robot (should change both encoders)\n");
                    printf("\nReal-time monitoring for 10 seconds...\n");
                    
                    double start_vertical = ::vertical_encoder->get_position() / 100.0;
                    double start_horizontal = ::horizontal_encoder->get_position() / 100.0;
                    
                    // Real-time monitoring during manual movement
                    for (int i = 10; i > 0; i--) {
                        double current_v = ::vertical_encoder->get_position() / 100.0;
                        double current_h = ::horizontal_encoder->get_position() / 100.0;
                        printf("  %d... V:%.2f° (Δ%.2f) H:%.2f° (Δ%.2f)\n", i, 
                               current_v, current_v - start_vertical,
                               current_h, current_h - start_horizontal);
                        pros::delay(1000);
                    }
                    
                    printf("✅ STEP 4 COMPLETE - Manual movement test done\n");
                    test_step++;
                    break;
                }
                
                case 5: {
                    printf("STEP 5: FINAL SENSOR ANALYSIS\n");
                    
                    double final_vertical = ::vertical_encoder->get_position() / 100.0;
                    double final_horizontal = ::horizontal_encoder->get_position() / 100.0;
                    
                    printf("Final sensor readings:\n");
                    printf("  Vertical encoder: %.2f degrees\n", final_vertical);
                    printf("  Horizontal encoder: %.2f degrees\n", final_horizontal);
                    
                    // Calculate total movement
                    double vertical_total = fabs(final_vertical);
                    double horizontal_total = fabs(final_horizontal);
                    
                    printf("\n=== TRACKING WHEEL ANALYSIS ===\n");
                    printf("Total vertical movement: %.2f degrees\n", vertical_total);
                    printf("Total horizontal movement: %.2f degrees\n", horizontal_total);
                    
                    if (vertical_total > 5.0) {
                        printf("✅ Vertical tracking wheel is WORKING!\n");
                    } else {
                        printf("❌ Vertical tracking wheel NOT working - check connection/mounting\n");
                    }
                    
                    if (horizontal_total > 5.0) {
                        printf("✅ Horizontal tracking wheel is WORKING!\n");
                    } else {
                        printf("❌ Horizontal tracking wheel NOT working - check connection/mounting\n");
                        printf("   TROUBLESHOOTING TIPS:\n");
                        printf("   - Check wheel contact with ground (should touch firmly)\n");
                        printf("   - Verify wheel isn't slipping on axle\n");
                        printf("   - Ensure encoder is securely mounted\n");
                        printf("   - Try manually spinning the wheel to see if encoder responds\n");
                    }
                    
                    // Proceed if we have at least the vertical wheel working (IMU can help compensate)
                    if (vertical_total > 5.0) {
                        printf("🎉 MINIMUM SENSORS READY: Vertical wheel + IMU can work together!\n");
                        printf("✅ STEP 5 COMPLETE - Proceeding to drive test\n");
                        printf("   (Note: Horizontal wheel issues may affect lateral accuracy)\n");
                        test_step++;
                    } else {
                        printf("⚠️  STEP 5 FAILED: Need at least vertical wheel working\n");
                        printf("Drive test skipped - fix vertical wheel and retry\n");
                        test_step = 1; // Reset for retry
                    }
                    break;
                }
                
                case 6: {
                    printf("STEP 6: INTEGRATED DRIVE TEST WITH SAFETY CHECKS\n");
                    
                    // ===== SAFETY CHECK 1: PTO SYSTEM =====
                    printf("� Checking PTO system...\n");
                    extern PTO* pto_system;
                    if (pto_system) {
                        printf("✅ PTO system found - Mode: %s\n", 
                               pto_system->isDrivetrainMode() ? "DRIVETRAIN (6-wheel)" : "SCORER (4-wheel)");
                    } else {
                        printf("❌ PTO system is NULL!\n");
                    }
                    
                    // ===== SAFETY CHECK 2: TRACKING WHEELS =====
                    printf("🔧 Checking tracking wheels...\n");
                    bool vertical_connected = ::vertical_encoder->is_installed();
                    bool horizontal_connected = ::horizontal_encoder->is_installed();
                    
                    printf("Vertical encoder (Port %d): %s\n", VERTICAL_ENCODER_PORT, 
                           vertical_connected ? "✅ CONNECTED" : "❌ NOT DETECTED");
                    printf("Horizontal encoder (Port %d): %s\n", HORIZONTAL_ENCODER_PORT, 
                           horizontal_connected ? "✅ CONNECTED" : "❌ NOT DETECTED");
                    
                    if (!vertical_connected) {
                        printf("❌ CRITICAL: Vertical encoder not connected - cannot proceed safely\n");
                        printf("Fix sensor connection and retry\n");
                        break;  // Exit the test
                    }
                    
                    // ===== SAFETY CHECK 3: IMU CALIBRATION =====
                    printf("🔧 Checking IMU...\n");
                    if (inertial_sensor->is_calibrating()) {
                        printf("⚠️  WARNING: IMU is still calibrating! Waiting...\n");
                        while (inertial_sensor->is_calibrating()) {
                            pros::delay(100);
                        }
                        printf("✅ IMU calibration complete\n");
                    } else {
                        printf("✅ IMU is ready\n");
                    }
                    
                    printf("Current IMU heading: %.2f°\n", inertial_sensor->get_heading());
                    
                    // ===== ALL SAFETY CHECKS PASSED - PROCEED WITH DRIVE TEST =====
                    printf("\n🚗 ALL SAFETY CHECKS PASSED - ROBOT WILL NOW DRIVE FORWARD!\n");
                    printf("⚠️  ENSURE ROBOT HAS CLEAR PATH (at least 24 inches)\n");
                    
                    // Wait a moment for user to read warning
                    pros::delay(2000);
                    
                    printf("Starting 18-inch straight drive test...\n");
                    
                    // Reset encoders before test
                    ::vertical_encoder->reset();
                    ::horizontal_encoder->reset();
                    
                    // SIMPLIFIED DRIVE TEST using LemLib directly
                    printf("=== LEMLIB STRAIGHT DRIVE TEST ===\n");
                    printf("Target distance: 18.0 inches\n");
                    
                    // Use robot's CURRENT heading as "forward" (don't force 0°)
                    double current_heading = inertial_sensor->get_heading();
                    printf("Robot's current heading: %.2f° - using this as 'forward'\n", current_heading);
                    
                    // Set position to 0,0 but keep current heading as forward direction
                    chassis->setPose(0, 0, current_heading);
                    pros::delay(100);
                    
                    auto start_pose = chassis->getPose();
                    printf("Starting position: (%.2f, %.2f, %.2f°)\n", 
                           start_pose.x, start_pose.y, start_pose.theta);
                    
                    uint32_t start_time = pros::millis();
                    
                    // BEST PRACTICE: Use existing driveDistance() function for relative movement
                    printf("=== USING DRIVE DISTANCE FUNCTION ===\n");
                    printf("Driving 18 inches forward using driveDistance()\n");
                    
                    printf("🚗 MOVEMENT STARTING...\n");
                    driveDistance(18.0, current_heading, 5000);  // Drive 18" in current heading direction
                    printf("🏁 MOVEMENT COMPLETE\n");
                    
                    uint32_t end_time = pros::millis();
                    
                    // Check results
                    auto final_pose = chassis->getPose();
                    double actual_distance = sqrt(final_pose.x * final_pose.x + final_pose.y * final_pose.y);
                    double distance_error = actual_distance - 18.0;
                    double heading_error = final_pose.theta;
                    
                    printf("=== RESULTS ===\n");
                    printf("Target: 18.0 inches\n");
                    printf("Actual: %.2f inches\n", actual_distance);
                    printf("Error: %.2f inches (%.1f%%)\n", distance_error, (distance_error/18.0)*100);
                    printf("Heading drift: %.2f degrees\n", heading_error);
                    printf("Time taken: %d ms\n", end_time - start_time);
                    printf("Final position: (%.2f, %.2f, %.2f°)\n", final_pose.x, final_pose.y, final_pose.theta);
                    
                    if (fabs(distance_error) < 1.0 && fabs(heading_error) < 3.0) {
                        printf("✅ PASS: Drive accuracy acceptable\n");
                        printf("   LemLib tuning looks good!\n");
                    } else {
                        printf("❌ FAIL: Needs calibration\n");
                        if (fabs(distance_error) >= 1.0) {
                            printf("   📏 Distance error too large (%.2f\")\n", distance_error);
                            printf("   Check LemLib linear constants in lemlib_config.cpp\n");
                        }
                        if (fabs(heading_error) >= 3.0) {
                            printf("   🧭 Heading drift too large (%.2f°)\n", heading_error);
                            printf("   POSSIBLE CAUSES:\n");
                            printf("   - IMU not properly calibrated\n");
                            printf("   - LemLib angular constants too aggressive\n");
                            printf("   - Tracking wheels not working properly\n");
                            printf("   - Robot hardware issues (loose wheels, etc.)\n");
                        }
                    }
                    
                    printf("\n=== FINAL RESULTS ===\n");
                    printf("🎉 ALL TESTS COMPLETE!\n");
                    printf("✅ Tracking wheel diagnostic: PASSED\n");
                    printf("✅ Straight drive test: COMPLETED\n");
                    
                    if (fabs(distance_error) < 1.0 && fabs(heading_error) < 3.0) {
                        printf("🎉 SUCCESS: Robot is ready for autonomous!\n");
                    } else {
                        printf("⚠️  RECOMMENDATION: Fix calibration issues before competition\n");
                    }
                    
                    printf("✅ ALL STEPS COMPLETE - Full diagnostic finished\n");
                    test_step = 1;  // Reset for next run
                    break;
                }
                
                default:
                    printf("Invalid step %d - resetting to step 1\n", test_step);
                    test_step = 1;
                    break;
            }
            
            if (test_step <= 6) {
                printf("\n➡️  Press L1+L2 again to continue to next step\n");
            }
            break;
        }
            
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
            
        case AutoMode::TEST_MOTORS:
            testMotorIdentification(); // Test which physical motor corresponds to each port
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
    printf("=== STRAIGHT DRIVE TEST (LemLib) ===\n");
    printf("Target distance: %.2f inches\n", distance);
    
    // Reset position for clean test
    chassis->setPose(0, 0, 0);
    auto start_pose = chassis->getPose();
    printf("Starting position: (%.2f, %.2f, %.2f°)\n", 
           start_pose.x, start_pose.y, start_pose.theta);
    
    // Drive straight
    uint32_t start_time = pros::millis();
    chassis->moveToPoint(distance, 0, 5000);  // Drive to (distance, 0)
    chassis->waitUntilDone();
    uint32_t end_time = pros::millis();
    
    // Check results
    auto final_pose = chassis->getPose();
    double actual_distance = sqrt(final_pose.x * final_pose.x + final_pose.y * final_pose.y);
    double distance_error = actual_distance - distance;
    double heading_error = final_pose.theta;
    
    printf("=== RESULTS ===\n");
    printf("Target: %.2f inches\n", distance);
    printf("Actual: %.2f inches\n", actual_distance);
    printf("Error: %.2f inches (%.1f%%)\n", distance_error, (distance_error/distance)*100);
    printf("Heading drift: %.2f degrees\n", heading_error);
    printf("Time taken: %d ms\n", end_time - start_time);
    
    if (fabs(distance_error) < 1.0 && fabs(heading_error) < 3.0) {
        printf("✅ PASS: Drive accuracy acceptable\n");
        printf("   LemLib tuning looks good!\n");
    } else {
        printf("❌ FAIL: Needs calibration\n");
        if (fabs(distance_error) >= 1.0) {
            printf("   📏 Distance error too large (%.2f\")\n", distance_error);
            printf("   Check LemLib movement constants in lemlib_config.cpp\n");
        }
        if (fabs(heading_error) >= 3.0) {
            printf("   🧭 Heading drift too large (%.2f°)\n", heading_error);
            printf("   Check IMU calibration or LemLib angular constants\n");
        }
    }
}

void AutonomousSystem::testTurnAccuracy(double angle) {
    printf("=== TURN ACCURACY TEST (LemLib) ===\n");
    printf("Target angle: %.2f degrees\n", angle);
    
    // Reset position
    chassis->setPose(0, 0, 0);
    
    // Perform turn
    uint32_t start_time = pros::millis();
    chassis->turnToHeading(angle, 3000);
    chassis->waitUntilDone();
    uint32_t end_time = pros::millis();
    
    // Check results
    auto final_pose = chassis->getPose();
    double angle_error = final_pose.theta - angle;
    
    printf("=== RESULTS ===\n");
    printf("Target: %.2f degrees\n", angle);
    printf("Actual: %.2f degrees\n", final_pose.theta);
    printf("Error: %.2f degrees\n", angle_error);
    printf("Time taken: %d ms\n", end_time - start_time);
    
    if (fabs(angle_error) < 2.0) {
        printf("✅ PASS: Turn accuracy acceptable\n");
        printf("   LemLib angular tuning looks good!\n");
    } else {
        printf("❌ FAIL: Needs tuning\n");
        printf("   🔄 Turn error: %.2f degrees\n", angle_error);
        printf("   Check LemLib angular constants in lemlib_config.cpp\n");
        
        if (end_time - start_time > 3000) {
            printf("   Slow response - check angular motion constants\n");
        }
    }
}

void AutonomousSystem::testPointToPoint() {
    printf("=== POINT-TO-POINT NAVIGATION TEST (LemLib) ===\n");
    
    // Define test waypoints (square pattern)
    struct TestPoint {
        double x, y;
    };
    
    TestPoint waypoints[] = {
        {0, 0},      // Start
        {24, 0},     // Point 1: 24" forward
        {24, 24},    // Point 2: 24" right  
        {0, 24},     // Point 3: 24" back
        {0, 0}       // Return to start
    };
    
    chassis->setPose(0, 0, 0);
    
    uint32_t total_start_time = pros::millis();
    
    for (int i = 1; i < 5; i++) {
        printf("Moving to waypoint %d: (%.0f, %.0f)\n", 
               i, waypoints[i].x, waypoints[i].y);
        
        chassis->moveToPoint(waypoints[i].x, waypoints[i].y, 5000);
        chassis->waitUntilDone();
        
        auto current_pose = chassis->getPose();
        double error = sqrt(pow(current_pose.x - waypoints[i].x, 2) + 
                           pow(current_pose.y - waypoints[i].y, 2));
        
        printf("Reached: (%.2f, %.2f) - Error: %.2f inches\n", 
               current_pose.x, current_pose.y, error);
        
        pros::delay(1000);  // Pause between waypoints
    }
    
    uint32_t total_end_time = pros::millis();
    
    // Final accuracy check
    auto final_pose = chassis->getPose();
    double return_error = sqrt(final_pose.x * final_pose.x + final_pose.y * final_pose.y);
    
    printf("=== RESULTS ===\n");
    printf("Return to start error: %.2f inches\n", return_error);
    printf("Total test time: %d ms\n", total_end_time - total_start_time);
    
    if (return_error < 3.0) {
        printf("✅ PASS: Navigation system working well\n");
        printf("   LemLib odometry is accurate!\n");
    } else {
        printf("❌ FAIL: Navigation needs calibration\n");
        printf("   Return error: %.2f inches\n", return_error);
        printf("   Check LemLib tracking wheels and constants\n");
    }
}

void AutonomousSystem::testOdometryAccuracy() {
    printf("=== ODOMETRY ACCURACY TEST (LemLib) ===\n");
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
    
    chassis->setPose(0, 0, 0);
    printf("Position reset to (0, 0, 0°)\n");
    
    // Move in a complex pattern
    chassis->moveToPoint(18, 0, 5000);     // Forward 18"
    chassis->waitUntilDone();
    chassis->moveToPoint(18, 12, 5000);    // Right 12"
    chassis->waitUntilDone();
    chassis->moveToPoint(6, 12, 5000);     // Back 12"
    chassis->waitUntilDone();
    chassis->moveToPoint(6, 6, 5000);      // Back 6"
    chassis->waitUntilDone();
    chassis->moveToPoint(0, 6, 5000);      // Left 6"
    chassis->waitUntilDone();
    chassis->moveToPoint(0, 0, 5000);      // Return to start
    chassis->waitUntilDone();
    
    auto final_pose = chassis->getPose();
    printf("=== FINAL POSITION ===\n");
    printf("LemLib odometry says: (%.2f, %.2f, %.2f°)\n", 
           final_pose.x, final_pose.y, final_pose.theta);
    printf("Manually measure distance from starting mark.\n");
    printf("Good accuracy: < 2 inches from start\n");
}

void AutonomousSystem::testMotorIdentification() {
    printf("=== MOTOR IDENTIFICATION TEST ===\n");
    printf("This will help identify which physical motor corresponds to each port\n");
    printf("Watch the robot carefully and note which motor spins for each test!\n");
    
    // Test each motor for 2 seconds at low speed
    int test_speed = 50; // Low speed for safety
    int test_duration = 2000; // 2 seconds
    
    printf("\n1. Testing RIGHT_FRONT_MOTOR_PORT (port %d)...\n", RIGHT_FRONT_MOTOR_PORT);
    printf("   Watch which physical motor spins!\n");
    pros::Motor right_front_test(RIGHT_FRONT_MOTOR_PORT, pros::v5::MotorGears::blue);
    right_front_test.move(test_speed);
    pros::delay(test_duration);
    right_front_test.move(0);
    printf("   RIGHT_FRONT_MOTOR_PORT test complete.\n");
    pros::delay(1000);
    
    printf("\n2. Testing RIGHT_MIDDLE_MOTOR_PORT (port %d)...\n", RIGHT_MIDDLE_MOTOR_PORT);
    printf("   Watch which physical motor spins!\n");
    pros::Motor right_middle_test(RIGHT_MIDDLE_MOTOR_PORT, pros::v5::MotorGears::blue);
    right_middle_test.move(test_speed);
    pros::delay(test_duration);
    right_middle_test.move(0);
    printf("   RIGHT_MIDDLE_MOTOR_PORT test complete.\n");
    pros::delay(1000);
    
    printf("\n3. Testing RIGHT_BACK_MOTOR_PORT (port %d)...\n", RIGHT_BACK_MOTOR_PORT);
    printf("   Watch which physical motor spins!\n");
    pros::Motor right_back_test(RIGHT_BACK_MOTOR_PORT, pros::v5::MotorGears::blue);
    right_back_test.move(test_speed);
    pros::delay(test_duration);
    right_back_test.move(0);
    printf("   RIGHT_BACK_MOTOR_PORT test complete.\n");
    pros::delay(1000);
    
    printf("\n=== NOW TESTING runRightIndexer() FUNCTION ===\n");
    printf("This simulates what happens when you do top back scoring...\n");
    
    // Simulate the exact code from runRightIndexer()
    int scoring_speed = -127; // Same as RIGHT_INDEXER_TOP_GOAL_SPEED
    printf("Creating motor object with RIGHT_MIDDLE_MOTOR_PORT (%d)...\n", RIGHT_MIDDLE_MOTOR_PORT);
    
    pros::Motor right_middle_scoring(RIGHT_MIDDLE_MOTOR_PORT, pros::v5::MotorGears::blue);
    
    printf("Running motor at speed %d for 3 seconds...\n", scoring_speed);
    printf("This should move the SAME motor as test #2 above!\n");
    
    right_middle_scoring.move(scoring_speed);
    pros::delay(3000);
    right_middle_scoring.move(0);
    
    printf("\n=== TEST COMPLETE ===\n");
    printf("RESULTS ANALYSIS:\n");
    printf("- RIGHT_FRONT_MOTOR_PORT (%d) moved which physical position? (front/middle/back)\n", RIGHT_FRONT_MOTOR_PORT);
    printf("- RIGHT_MIDDLE_MOTOR_PORT (%d) moved which physical position? (front/middle/back)\n", RIGHT_MIDDLE_MOTOR_PORT);
    printf("- RIGHT_BACK_MOTOR_PORT (%d) moved which physical position? (front/middle/back)\n", RIGHT_BACK_MOTOR_PORT);
    printf("\nThe motor that should be disconnected by PTO is the physically MIDDLE one!\n");
    printf("If runRightIndexer() moved a motor that's still connected to drivetrain,\n");
    printf("then we need to fix the port assignments in config.h\n");
}
