/**
 * Motor identification test
 * This will help identify which physical motor corresponds to each port
 */

#include "main.h"

void test_individual_motors() {
    printf("=== MOTOR IDENTIFICATION TEST ===\n");
    printf("Testing each right-side motor individually...\n");
    
    // Test each motor for 2 seconds at low speed
    int test_speed = 50; // Low speed for safety
    int test_duration = 2000; // 2 seconds
    
    printf("\n1. Testing RIGHT_FRONT_MOTOR_PORT (port %d)...\n", RIGHT_FRONT_MOTOR_PORT);
    printf("   Watch which physical motor spins!\n");
    pros::Motor right_front_test(RIGHT_FRONT_MOTOR_PORT);
    right_front_test.move(test_speed);
    pros::delay(test_duration);
    right_front_test.move(0);
    printf("   RIGHT_FRONT_MOTOR_PORT test complete.\n");
    pros::delay(1000);
    
    printf("\n2. Testing RIGHT_MIDDLE_MOTOR_PORT (port %d)...\n", RIGHT_MIDDLE_MOTOR_PORT);
    printf("   Watch which physical motor spins!\n");
    pros::Motor right_middle_test(RIGHT_MIDDLE_MOTOR_PORT);
    right_middle_test.move(test_speed);
    pros::delay(test_duration);
    right_middle_test.move(0);
    printf("   RIGHT_MIDDLE_MOTOR_PORT test complete.\n");
    pros::delay(1000);
    
    printf("\n3. Testing RIGHT_BACK_MOTOR_PORT (port %d)...\n", RIGHT_BACK_MOTOR_PORT);
    printf("   Watch which physical motor spins!\n");
    pros::Motor right_back_test(RIGHT_BACK_MOTOR_PORT);
    right_back_test.move(test_speed);
    pros::delay(test_duration);
    right_back_test.move(0);
    printf("   RIGHT_BACK_MOTOR_PORT test complete.\n");
    pros::delay(1000);
    
    printf("\n=== TEST COMPLETE ===\n");
    printf("Based on what you observed:\n");
    printf("- RIGHT_FRONT_MOTOR_PORT (%d) moved which physical motor?\n", RIGHT_FRONT_MOTOR_PORT);
    printf("- RIGHT_MIDDLE_MOTOR_PORT (%d) moved which physical motor?\n", RIGHT_MIDDLE_MOTOR_PORT);
    printf("- RIGHT_BACK_MOTOR_PORT (%d) moved which physical motor?\n", RIGHT_BACK_MOTOR_PORT);
    printf("\nThe RIGHT_MIDDLE should be the one that gets disconnected by PTO!\n");
}

void test_runRightIndexer_function() {
    printf("\n=== TESTING runRightIndexer() FUNCTION ===\n");
    printf("This simulates what happens when you do top back scoring...\n");
    
    // Simulate the exact code from runRightIndexer()
    int speed = -127; // Same as RIGHT_INDEXER_TOP_GOAL_SPEED
    printf("Creating motor object with RIGHT_MIDDLE_MOTOR_PORT (%d)...\n", RIGHT_MIDDLE_MOTOR_PORT);
    
    pros::Motor right_middle(RIGHT_MIDDLE_MOTOR_PORT, pros::v5::MotorGears::blue);
    
    printf("Running motor at speed %d for 3 seconds...\n", speed);
    printf("Watch which physical motor moves!\n");
    
    right_middle.move(speed);
    pros::delay(3000);
    right_middle.move(0);
    
    printf("runRightIndexer() simulation complete.\n");
    printf("This should have moved the same motor that will be used for scoring.\n");
}