/**
 * System Tests - All Testing Functions
 * Centralized test suite for motors, sensors, and odometry
 */

#include "main.h"
#include "config.h"
#include "indexer.h"
#include "color_sensor.h"
#include "intake.h"
#include "lemlib/api.hpp"
#include <cmath>

// External system references (declared in main.h)
extern pros::Controller* master;
extern Intake* intake_system;
extern IndexerSystem* indexer_system;
extern ColorSensorSystem* color_sensor_system;
extern lemlib::Chassis* chassis;  // LemLib chassis for odometry tests

// =============================================================================
// LEMLIB ODOMETRY TESTS (Local functions)
// =============================================================================

/**
 * Test straight drive accuracy using LemLib
 */
static void testStraightDrive(double distance, lemlib::Chassis* chassis) {
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

/**
 * Test turn accuracy using LemLib
 */
static void testTurnAccuracy(double angle, lemlib::Chassis* chassis) {
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

/**
 * Test point-to-point navigation using LemLib
 */
static void testPointToPoint(lemlib::Chassis* chassis) {
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

/**
 * Test odometry accuracy with manual verification
 */
static void testOdometryAccuracy(lemlib::Chassis* chassis) {
    printf("=== ODOMETRY ACCURACY TEST (LemLib) ===\n");
    printf("This test requires manual measurement!\n");
    printf("1. Mark robot's current position\n");
    printf("2. Press any controller button to continue...\n");
    
    // Wait for controller input
    while (!master->get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
           !master->get_digital(pros::E_CONTROLLER_DIGITAL_B) &&
           !master->get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
           !master->get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
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

// =============================================================================
// HARDWARE TESTS (Local functions)
// =============================================================================


/**
 * Test 14: Motor Identification Test
 * Runs each motor individually to help identify ports
 */
static void testMotorIdentification() {
    printf("\n🔧 MOTOR IDENTIFICATION TEST\n");
    printf("Each motor will run for 2 seconds at 50%% speed\n");
    printf("Watch the robot to identify which motor is which!\n\n");
    
    master->set_text(0, 0, "MOTOR ID TEST");
    master->set_text(1, 0, "UP to stop");
    
    // Test configuration
    struct MotorTest {
        int port;
        const char* name;
        int speed;
    };
    
    MotorTest motors[] = {
        {LEFT_FRONT_MOTOR_PORT, "LEFT FRONT", 50},
        {LEFT_MIDDLE_MOTOR_PORT, "LEFT MIDDLE", 50},
        {LEFT_BACK_MOTOR_PORT, "LEFT BACK", 50},
        {RIGHT_FRONT_MOTOR_PORT, "RIGHT FRONT", 50},
        {RIGHT_MIDDLE_MOTOR_PORT, "RIGHT MIDDLE", 50},
        {RIGHT_BACK_MOTOR_PORT, "RIGHT BACK", 50},
        {INPUT_MOTOR_PORT, "INPUT/INTAKE", 50},
        {TOP_INDEXER_PORT, "TOP INDEXER", 50},
        {FRONT_LOADER_MOTOR_PORT, "FRONT LOADER", 50}
    };
    
    for (const auto& motor : motors) {
        if (master->get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            printf("❌ Test stopped by user\n");
            break;
        }
        
        printf("Testing: %s (Port %d)\n", motor.name, motor.port);
        master->set_text(0, 0, motor.name);
        
        // Create and run motor
        pros::Motor test_motor(motor.port);
        test_motor.move(motor.speed);
        pros::delay(2000);
        test_motor.brake();
        
        pros::delay(500);  // Pause between motors
    }
    
    printf("\n✅ Motor identification test complete\n");
    master->set_text(0, 0, "Test Complete");
}

/**
 * Test 15: Color Sorting System Test
 * Tests color detection, collection, and ejection
 */
static void testColorSorter() {
    printf("\n🎨 COLOR SORTING SYSTEM TEST 🎨\n");
    printf("Testing color sensor detection, ball collection, and ejection\n");
    printf("Controller Controls:\n");
    printf("  UP Button: Stop test\n");
    printf("  LEFT Stick Left: Switch to COLLECT_RED (eject blue)\n");
    printf("  RIGHT Stick Right: Switch to COLLECT_BLUE (eject red)\n");
    printf("  X Button: Switch to COLLECT_ALL (no ejection)\n");
    printf("  B Button: Switch to EJECT_ALL (testing only)\n\n");
    
    // Reset statistics for clean test
    color_sensor_system->resetStatistics();
    // indexer_system->resetStorageBallCount(); // Removed - storage counting no longer exists
    
    // Set to COLLECT_ALL mode initially (test detection and counters first)
    color_sensor_system->setSortingMode(SortingMode::COLLECT_ALL);
    printf("🎯 Starting in COLLECT_ALL mode - all balls will be collected\n");
    printf("   This mode tests detection and counter functionality\n");
    printf("   Feed RED and BLUE balls to verify counting!\n\n");
    
    // Display on controller
    master->set_text(0, 0, "COLLECT ALL");
    master->set_text(1, 0, "No Eject");
    
    // Test loop
    while (true) {
        // Check for stop command
        if (master->get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            printf("\n⏹️ Test stopped by user\n");
            break;
        }
        
        // Mode switching controls
        if (master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) < -64) {
            color_sensor_system->setSortingMode(SortingMode::COLLECT_RED);
            printf("🔴 Mode: COLLECT_RED (eject blue)\n");
            master->set_text(0, 0, "EJECT BLUE");
            master->set_text(1, 0, "Collect RED");
            pros::delay(300);  // Debounce
        }
        
        if (master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) > 64) {
            color_sensor_system->setSortingMode(SortingMode::COLLECT_BLUE);
            printf("🔵 Mode: COLLECT_BLUE (eject red)\n");
            master->set_text(0, 0, "EJECT RED");
            master->set_text(1, 0, "Collect BLUE");
            pros::delay(300);  // Debounce
        }
        
        if (master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            color_sensor_system->setSortingMode(SortingMode::COLLECT_ALL);
            printf("⚪ Mode: COLLECT_ALL (no ejection)\n");
            master->set_text(0, 0, "COLLECT ALL");
            master->set_text(1, 0, "No Eject");
        }
        
        if (master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            color_sensor_system->setSortingMode(SortingMode::EJECT_ALL);
            printf("🚫 Mode: EJECT_ALL (testing)\n");
            master->set_text(0, 0, "EJECT ALL");
            master->set_text(1, 0, "TEST MODE");
        }
        
        // Update systems
        color_sensor_system->update();
        indexer_system->update(*master);
        
        // Display statistics periodically (every 2 seconds)
        static uint32_t last_stats_time = 0;
        if (pros::millis() - last_stats_time > 2000) {
            // printf("📊 Storage: %d/3 balls\n\n", 
            //        indexer_system->getStorageBallCount()); // Removed - storage counting no longer exists
            printf("📊 Systems running\n\n");
            last_stats_time = pros::millis();
        }
        
        pros::delay(20);
    }
    
    // Print final statistics
    printf("\n📊 FINAL TEST STATISTICS:\n");
    // printf("Final storage: %d/3 balls\n", 
    //        indexer_system->getStorageBallCount()); // Removed - storage counting no longer exists
    
    master->set_text(0, 0, "Test Complete");
    master->set_text(1, 0, "");
}

// =============================================================================
// CENTRAL TEST RUNNER
// =============================================================================

/**
 * Main test entry point - called from AutonomousSystem::run()
 * Routes to appropriate test based on AutoMode
 * 
 * @param mode The test mode to execute
 * @param chassis Pointer to LemLib chassis (for odometry tests)
 */
void runTest(AutoMode mode, lemlib::Chassis* chassis) {
    printf("\n" "════════════════════════════════════════\n");
    printf("         TEST MODE ACTIVATED\n");
    printf("════════════════════════════════════════\n\n");
    
    switch (mode) {
        case AutoMode::TEST_DRIVE:
            printf("📏 TEST: Straight Drive (18 inches)\n");
            printf("Current PID gains: P=%.2f, I=%.3f, D=%.2f\n", DRIVE_KP, DRIVE_KI, DRIVE_KD);
            testStraightDrive(18.0, chassis);
            break;
            
        case AutoMode::TEST_TURN:
            printf("🔄 TEST: Turn 90 degrees\n");
            printf("Current PID gains: P=%.2f, I=%.3f, D=%.2f\n", TURN_KP, TURN_KI, TURN_KD);
            testTurnAccuracy(90.0, chassis);
            break;
            
        case AutoMode::TEST_NAVIGATION:
            testPointToPoint(chassis);
            break;
            
        case AutoMode::TEST_ODOMETRY:
            testOdometryAccuracy(chassis);
            break;
            
        case AutoMode::TEST_MOTORS:
            testMotorIdentification();
            break;
            
        case AutoMode::TEST_COLOR_SORTER:
            printf("🎨 TEST: Color Sorter & Ejection\n");
            testColorSorter();
            break;
            
        default:
            printf("⚠️ Unknown test mode\n");
            break;
    }
    
    printf("\n" "════════════════════════════════════════\n");
    printf("         TEST COMPLETE\n");
    printf("════════════════════════════════════════\n\n");
}
