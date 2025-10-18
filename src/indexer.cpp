/**
 * \file indexer.cpp
 *
 * Indexer and scoring system implementation.
 * Manages ball intake, indexing, and scoring for both front and back directions.
 */

#include "indexer.h"

IndexerSystem::IndexerSystem(PTO* pto) 
    : input_motor(INPUT_MOTOR_PORT, DRIVETRAIN_GEARSET),
      top_indexer(TOP_INDEXER_PORT, DRIVETRAIN_GEARSET),
      front_flap(FRONT_FLAP_PNEUMATIC, false),
      pto_system(pto),
      current_mode(ScoringMode::NONE),
      last_direction(ExecutionDirection::NONE),
      scoring_active(false),
      scoring_start_time(0),
      input_start_time(0),
      input_motor_active(false),
      last_collection_button(false),
      last_mid_goal_button(false),
      last_low_goal_button(false),
      last_top_goal_button(false),
      last_front_execute_button(false),
      last_back_execute_button(false) {
    
    // Set motor brake modes for precise control
    input_motor.set_brake_mode(DRIVETRAIN_BRAKE_MODE);
    top_indexer.set_brake_mode(DRIVETRAIN_BRAKE_MODE);
    
    // Ensure all motors start stopped
    stopAll();
}

void IndexerSystem::setCollectionMode() {
    current_mode = ScoringMode::COLLECTION;
    
    // Debug output to console only
    printf("DEBUG: Set COLLECTION mode\n");
    
    // Send to controller if available
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (master.is_connected()) {
        master.print(0, 0, "COLLECTION");
    }
}

void IndexerSystem::setMidGoalMode() {
    current_mode = ScoringMode::MID_GOAL;
    
    // Debug output to console only
    printf("DEBUG: Set MID GOAL mode\n");
    
    // Send to controller
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (master.is_connected()) {
        master.print(0, 0, "MID GOAL");
    }
}

void IndexerSystem::setLowGoalMode() {
    current_mode = ScoringMode::LOW_GOAL;
    
    // Debug output
    // LCD call removed to prevent rendering conflicts
    printf("DEBUG: Set LOW GOAL mode\n");
    
    // Send to controller
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (master.is_connected()) {
        master.print(0, 0, "LOW GOAL");
    }
}

void IndexerSystem::setTopGoalMode() {
    current_mode = ScoringMode::TOP_GOAL;
    
    // Debug output
    // LCD call removed to prevent rendering conflicts
    printf("DEBUG: Set TOP GOAL mode\n");
    
    // Send to controller
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (master.is_connected()) {
        master.print(0, 0, "TOP GOAL");
    }
}

void IndexerSystem::executeFront() {
    printf("DEBUG: executeFront() called with mode: %d\n", (int)current_mode);
    
    // Can't execute without mode selected
    if (current_mode == ScoringMode::NONE) {
        printf("DEBUG: No mode selected\n");
        // LCD call removed to prevent rendering conflicts
        pros::Controller master(pros::E_CONTROLLER_MASTER);
        if (master.is_connected()) {
            master.print(1, 0, "Need Mode");
        }
        return;
    }
    
    // Stop any currently running sequence
    if (scoring_active) {
        printf("DEBUG: Stopping previous sequence\n");
        stopAll();
    }
    
    // Set last direction for tracking
    last_direction = ExecutionDirection::FRONT;
    
    // For low goal mode, we don't need pneumatics, so skip delays
    if (current_mode == ScoringMode::TOP_GOAL) {
        // IMPORTANT: Open front flap for scoring
        openFrontFlap();
        // Reduced delay to minimize blocking
        pros::delay(50); // Give pneumatics time to actuate
    } else {
        // Ensure front flap is closed for other modes
        closeFrontFlap();
    }
    if (current_mode != ScoringMode::LOW_GOAL) {
        // Ensure PTO is in scorer mode for front indexer (left middle motor)
        if (pto_system && pto_system->isDrivetrainMode()) {
            pto_system->setScorerMode();
            pros::delay(50); // Give pneumatics time to actuate
        }
    }
    
    // Execute based on mode
    switch (current_mode) {
        case ScoringMode::COLLECTION:
            printf("DEBUG: FRONT Collection - Left middle motor: %d\n", LEFT_INDEXER_FRONT_COLLECTION_SPEED);
            runLeftIndexer(LEFT_INDEXER_FRONT_COLLECTION_SPEED); // Direct speed for front collection
            runRightIndexer(RIGHT_INDEXER_COLLECTION_SPEED); // Direct speed for back collection
            runTopIndexer(TOP_INDEXER_FRONT_SPEED);  // Direct speed for top indexer front
            startInput(); // Start intake motor for collection
            // LCD call removed to prevent rendering conflicts
            break;
            
        case ScoringMode::MID_GOAL:
            printf("DEBUG: FRONT Mid Goal - Left middle motor: %d\n", LEFT_INDEXER_FRONT_MID_GOAL_SPEED);
            runLeftIndexer(LEFT_INDEXER_FRONT_MID_GOAL_SPEED); // Direct speed for front mid goal
            startInput(); // Input motor runs in all scoring modes
            // LCD call removed to prevent rendering conflicts
            break;
            
        case ScoringMode::LOW_GOAL:
            printf("DEBUG: FRONT Low Goal - Only intake motor reverse: %d\n", INPUT_MOTOR_REVERSE_SPEED);
            startInputReverse(); // Only run intake motor in reverse for low goal
            // LCD call removed to prevent rendering conflicts
            break;
            
        case ScoringMode::TOP_GOAL:
            printf("DEBUG: FRONT Top Goal - Left middle + top indexer: %d, %d\n", LEFT_INDEXER_FRONT_TOP_GOAL_SPEED, TOP_INDEXER_FRONT_SPEED);
            runLeftIndexer(LEFT_INDEXER_FRONT_TOP_GOAL_SPEED); // Direct speed for front top goal
            runTopIndexer(TOP_INDEXER_FRONT_SPEED);  // Direct speed for top indexer front
            runRightIndexer(RIGHT_INDEXER_COLLECTION_SPEED); // Direct speed for back collection
            startInput(); // Input motor runs in all scoring modes
            // LCD call removed to prevent rendering conflicts
            break;
            
        case ScoringMode::NONE:
        default:
            return; // Already handled above
    }
    
    // Start sequence timer
    scoring_active = true;
    scoring_start_time = pros::millis();
    
    // Controller feedback
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (master.is_connected()) {
        master.print(1, 0, "FRONT %s", getModeString());
    }
}

void IndexerSystem::executeBack() {
    printf("DEBUG: executeBack() called with mode: %d\n", (int)current_mode);
    
    // Can't execute without mode selected
    if (current_mode == ScoringMode::NONE) {
        printf("DEBUG: No mode selected\n");
        // LCD call removed to prevent rendering conflicts
        pros::Controller master(pros::E_CONTROLLER_MASTER);
        if (master.is_connected()) {
            master.print(1, 0, "Need Mode");
        }
        return;
    }
    
    // Stop any currently running sequence
    if (scoring_active) {
        printf("DEBUG: Stopping previous sequence\n");
        stopAll();
    }
    
    // Set last direction for tracking
    last_direction = ExecutionDirection::BACK;
    
    // For low goal mode, we don't need PTO, so skip delays
    if (current_mode != ScoringMode::LOW_GOAL) {
        // Ensure PTO is in scorer mode for back indexer
        if (pto_system && pto_system->isDrivetrainMode()) {
            pto_system->setScorerMode();
            pros::delay(50); // Reduced delay to minimize blocking
        }
    }
    
    // Execute based on mode
    switch (current_mode) {
        case ScoringMode::COLLECTION:
            printf("DEBUG: BACK Collection - Right: %d, Left helper: %d\n", RIGHT_INDEXER_COLLECTION_SPEED, LEFT_INDEXER_BACK_COLLECTION_SPEED);
            runRightIndexer(RIGHT_INDEXER_COLLECTION_SPEED); // Direct speed for back collection
            runLeftIndexer(LEFT_INDEXER_BACK_COLLECTION_SPEED); // Left motor helps bring ball upwards
            startInput(); // Start intake motor for collection
            // LCD call removed to prevent rendering conflicts
            break;
            
        case ScoringMode::MID_GOAL:
            printf("DEBUG: BACK Mid Goal - Right: %d, Left helper: %d\n", RIGHT_INDEXER_MID_GOAL_SPEED, LEFT_INDEXER_BACK_MID_GOAL_SPEED);
            runRightIndexer(RIGHT_INDEXER_MID_GOAL_SPEED); // Direct speed for back mid goal
            runLeftIndexer(LEFT_INDEXER_BACK_MID_GOAL_SPEED); // Left motor helps bring ball upwards
            startInput(); // Input motor runs in all scoring modes
            // LCD call removed to prevent rendering conflicts
            break;
            
        case ScoringMode::LOW_GOAL:
            printf("DEBUG: BACK Low Goal - Only intake motor reverse: %d\n", INPUT_MOTOR_REVERSE_SPEED);
            startInputReverse(); // Only run intake motor in reverse for low goal
            // LCD call removed to prevent rendering conflicts
            break;
            
        case ScoringMode::TOP_GOAL:
            printf("DEBUG: BACK Top Goal - Right: %d, Top: %d, Left helper: %d\n", RIGHT_INDEXER_TOP_GOAL_SPEED, TOP_INDEXER_BACK_SPEED, LEFT_INDEXER_BACK_TOP_GOAL_SPEED);
            runRightIndexer(RIGHT_INDEXER_TOP_GOAL_SPEED); // Direct speed for back top goal
            runTopIndexer(TOP_INDEXER_BACK_SPEED); // Direct speed for top indexer back
            runLeftIndexer(LEFT_INDEXER_BACK_TOP_GOAL_SPEED); // Left motor helps bring ball upwards
            startInput(); // Input motor runs in all scoring modes
            // LCD call removed to prevent rendering conflicts
            break;
            
        case ScoringMode::NONE:
        default:
            return; // Already handled above
    }
    
    // Start sequence timer
    scoring_active = true;
    scoring_start_time = pros::millis();
    
    // Controller feedback
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (master.is_connected()) {
        master.print(1, 0, "BACK %s", getModeString());
    }
}

void IndexerSystem::openFrontFlap() {
    front_flap.set_value(FRONT_FLAP_OPEN);
    printf("DEBUG: Front flap OPENED for scoring\n");
}

void IndexerSystem::closeFrontFlap() {
    front_flap.set_value(FRONT_FLAP_CLOSED);
    printf("DEBUG: Front flap CLOSED to hold balls\n");
    // LCD call removed to prevent rendering conflicts
}

void IndexerSystem::startInput() {
    if (!input_motor_active) {
        printf("DEBUG: Starting input motor at %d RPM\n", INPUT_MOTOR_SPEED);
        input_motor.move_velocity(INPUT_MOTOR_SPEED);
        input_motor_active = true;
        input_start_time = pros::millis();
        
        // LCD call removed to prevent rendering conflicts
        printf("DEBUG: Input motor started successfully\n");
    } else {
        printf("DEBUG: Input motor already active\n");
    }
}

void IndexerSystem::startInputReverse() {
    if (!input_motor_active) {
        printf("DEBUG: Starting input motor in REVERSE at %d RPM\n", INPUT_MOTOR_REVERSE_SPEED);
        input_motor.move_velocity(INPUT_MOTOR_REVERSE_SPEED);
        input_motor_active = true;
        input_start_time = pros::millis();
        
        // LCD call removed to prevent rendering conflicts
        printf("DEBUG: Input motor reverse started successfully\n");
    } else {
        printf("DEBUG: Input motor already active\n");
    }
}

void IndexerSystem::stopInput() {
    if (input_motor_active) {
        input_motor.move_velocity(0);
        input_motor_active = false;
        
        // LCD call removed to prevent rendering conflicts
    }
}

void IndexerSystem::stopAll() {
    printf("DEBUG: stopAll() called - resetting all motors and state\n");
    
    // Stop all motors explicitly
    input_motor.move_velocity(0);
    input_motor.move(0);  // Double-stop to ensure it's off
    
    stopLeftIndexer();   // Stop left middle motor (front)
    stopRightIndexer();  // Stop right middle motor (back)
    stopTopIndexer();    // Stop top indexer motor
    
    // IMPORTANT: Close front flap when stopping to hold balls
    closeFrontFlap();
    
    // Reset state completely to ensure system doesn't get stuck
    scoring_active = false;
    input_motor_active = false;
    last_direction = ExecutionDirection::NONE;  // Reset direction to prevent confusion
    
    // LCD call removed to prevent rendering conflicts
    printf("DEBUG: All state reset - scoring_active: %d, input_active: %d, direction: %d\n", 
           scoring_active, input_motor_active, (int)last_direction);
}

ScoringMode IndexerSystem::getCurrentMode() const {
    return current_mode;
}

ExecutionDirection IndexerSystem::getLastDirection() const {
    return last_direction;
}

bool IndexerSystem::isScoringActive() const {
    return scoring_active;
}

bool IndexerSystem::isInputActive() const {
    return input_motor_active;
}

void IndexerSystem::update(pros::Controller& controller) {
    // Debug: Print that update is being called
    static int update_counter = 0;
    update_counter++;
    if (update_counter % 100 == 0) {  // Every 2 seconds (50Hz * 100 = 2s)
        printf("DEBUG: IndexerSystem::update() called %d times\n", update_counter);
        
        if (controller.is_connected()) {
            controller.print(1, 0, "Updates: %d", update_counter);
        }
    }
    
    // Get current button states for new control scheme
    bool current_collection_button = controller.get_digital(COLLECTION_MODE_BUTTON);     // Y
    bool current_mid_goal_button = controller.get_digital(MID_GOAL_BUTTON);             // A
    bool current_low_goal_button = controller.get_digital(LOW_GOAL_BUTTON);             // B
    bool current_top_goal_button = controller.get_digital(TOP_GOAL_BUTTON);             // X
    bool current_front_execute_button = controller.get_digital(FRONT_EXECUTE_BUTTON);   // R2
    bool current_back_execute_button = controller.get_digital(BACK_EXECUTE_BUTTON);     // R1
    
    // Debug: Print button states when any button is pressed
    if (current_collection_button || current_mid_goal_button || current_low_goal_button || 
        current_top_goal_button || current_front_execute_button || current_back_execute_button) {
        printf("DEBUG: Buttons - Y:%d A:%d B:%d X:%d R2:%d R1:%d\n", 
               current_collection_button, current_mid_goal_button, current_low_goal_button,
               current_top_goal_button, current_front_execute_button, current_back_execute_button);
    }
    
    // Handle mode selection (rising edge detection)
    if (current_collection_button && !last_collection_button) {
        printf("DEBUG: Y (COLLECTION) button pressed!\n");
        setCollectionMode();
        controller.rumble(".");
        if (controller.is_connected()) {
            controller.print(2, 0, "Y COLLECTION");
        }
    }
    
    if (current_mid_goal_button && !last_mid_goal_button) {
        printf("DEBUG: A (MID GOAL) button pressed!\n");
        setMidGoalMode();
        controller.rumble(".");
        if (controller.is_connected()) {
            controller.print(2, 0, "A MID GOAL");
        }
    }
    
    if (current_low_goal_button && !last_low_goal_button) {
        printf("DEBUG: B (LOW GOAL) button pressed!\n");
        setLowGoalMode();
        controller.rumble(".");
        if (controller.is_connected()) {
            controller.print(2, 0, "B LOW GOAL");
        }
    }
    
    if (current_top_goal_button && !last_top_goal_button) {
        printf("DEBUG: X (TOP GOAL) button pressed!\n");
        setTopGoalMode();
        controller.rumble(".");
        if (controller.is_connected()) {
            controller.print(2, 0, "X TOP GOAL");
        }
    }
    
    // Handle execution with TOGGLE functionality (rising edge detection)
    if (current_front_execute_button && !last_front_execute_button) {
        printf("DEBUG: R2 (FRONT EXECUTE) button pressed!\n");
        printf("DEBUG: Current state - scoring_active: %d, last_direction: %d\n", scoring_active, (int)last_direction);
        
        // TOGGLE: If already scoring front, stop it. Otherwise start front execution.
        if (scoring_active && last_direction == ExecutionDirection::FRONT) {
            printf("DEBUG: R2 pressed again - STOPPING front execution\n");
            stopAll();
            controller.rumble("---"); // Long rumble for stop
            if (controller.is_connected()) {
                controller.print(2, 0, "R2 STOP");
            }
        } else {
            printf("DEBUG: R2 pressed - STARTING front execution\n");
            executeFront();
            controller.rumble(".."); // Double rumble for start
            if (controller.is_connected()) {
                controller.print(2, 0, "R2 START");
            }
        }
    }
    
    if (current_back_execute_button && !last_back_execute_button) {
        printf("DEBUG: R1 (BACK EXECUTE) button pressed!\n");
        printf("DEBUG: Current state - scoring_active: %d, last_direction: %d\n", scoring_active, (int)last_direction);
        
        // TOGGLE: If already scoring back, stop it. Otherwise start back execution.
        if (scoring_active && last_direction == ExecutionDirection::BACK) {
            printf("DEBUG: R1 pressed again - STOPPING back execution\n");
            stopAll();
            controller.rumble("---"); // Long rumble for stop
            if (controller.is_connected()) {
                controller.print(2, 0, "R1 STOP");
            }
        } else {
            printf("DEBUG: R1 pressed - STARTING back execution\n");
            executeBack();
            controller.rumble(".."); // Double rumble for start
            if (controller.is_connected()) {
                controller.print(2, 0, "R1 START");
            }
        }
    }
    
    // Testing controls for individual indexers
    static bool last_left_test_button = false;
    static bool last_right_test_button = false;
    bool current_left_test_button = controller.get_digital(LEFT_INDEXER_TEST_BUTTON);
    bool current_right_test_button = controller.get_digital(RIGHT_INDEXER_TEST_BUTTON);
    
    // Left indexer testing (Y button)
    if (current_left_test_button && !last_left_test_button) {
        // Toggle left indexer: forward speed first press, reverse second press, stop third press
        static int left_test_state = 0;
        left_test_state = (left_test_state + 1) % 3;
        
        switch (left_test_state) {
            case 0: stopLeftIndexer(); break;
            case 1: testLeftIndexer(100); break;  // Forward 100 RPM
            case 2: testLeftIndexer(-100); break; // Reverse 100 RPM
        }
        controller.rumble(".");
    }
    
    // Right indexer testing (UP button)  
    if (current_right_test_button && !last_right_test_button) {
        // Toggle right indexer: forward speed first press, reverse second press, stop third press
        static int right_test_state = 0;
        right_test_state = (right_test_state + 1) % 3;
        
        switch (right_test_state) {
            case 0: stopRightIndexer(); break;
            case 1: testRightIndexer(100); break;  // Forward 100 RPM
            case 2: testRightIndexer(-100); break; // Reverse 100 RPM
        }
        controller.rumble(".");
    }
    
    // Update test button states
    last_left_test_button = current_left_test_button;
    last_right_test_button = current_right_test_button;
    
    // IMPORTANT: Add timeout mechanism for low goal mode to prevent system from getting stuck
    if (scoring_active && current_mode == ScoringMode::LOW_GOAL) {
        // Automatic timeout for low goal mode after 3 seconds
        if (pros::millis() - scoring_start_time > 3000) {
            printf("DEBUG: Low goal mode timeout - automatically stopping\n");
            stopAll();
            // LCD call removed to prevent rendering conflicts
            
            // Notify controller
            if (controller.is_connected()) {
                controller.print(2, 0, "LOW TIMEOUT");
                controller.rumble("...");
            }
        }
    }
    
    // Emergency stop: If any execution button is held for more than 5 seconds, force stop
    if (scoring_active && (pros::millis() - scoring_start_time > 5000)) {
        printf("DEBUG: Emergency timeout - force stopping all operations\n");
        stopAll();
        // LCD call removed to prevent rendering conflicts
        
        if (controller.is_connected()) {
            controller.print(2, 0, "EMERGENCY STOP");
            controller.rumble("---");
        }
    }
    
    // Update last button states for next iteration
    last_collection_button = current_collection_button;
    last_mid_goal_button = current_mid_goal_button;
    last_low_goal_button = current_low_goal_button;
    last_top_goal_button = current_top_goal_button;
    last_front_execute_button = current_front_execute_button;
    last_back_execute_button = current_back_execute_button;
}

const char* IndexerSystem::getModeString() const {
    switch (current_mode) {
        case ScoringMode::COLLECTION:  return "COLLECTION";
        case ScoringMode::MID_GOAL:    return "MID GOAL";
        case ScoringMode::LOW_GOAL:    return "LOW GOAL";
        case ScoringMode::TOP_GOAL:    return "TOP GOAL";
        case ScoringMode::NONE:        return "NONE";
        default: return "UNKNOWN";
    }
}

const char* IndexerSystem::getDirectionString() const {
    switch (last_direction) {
        case ExecutionDirection::FRONT: return "FRONT";
        case ExecutionDirection::BACK:  return "BACK";
        case ExecutionDirection::NONE:  return "NONE";
        default: return "UNKNOWN";
    }
}

void IndexerSystem::runLeftIndexer(int speed) {
    // Left indexer uses the LEFT middle wheel via PTO for front storage/scoring
    printf("DEBUG: runLeftIndexer() called with speed: %d\n", speed);
    
    // Create motor object for LEFT middle wheel WITHOUT automatic reversal for direct control
    pros::Motor left_middle(LEFT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    
    // Run the left middle wheel for front indexer with direct speed control
    left_middle.move_velocity(speed);
    printf("DEBUG: Left middle motor (front indexer) direct speed: %d\n", speed);
}

void IndexerSystem::runRightIndexer(int speed) {
    // Right indexer uses the RIGHT middle wheel via PTO for back scoring
    printf("DEBUG: runRightIndexer() called with speed: %d\n", speed);
    
    // Create motor object for RIGHT middle wheel WITHOUT automatic reversal for direct control
    pros::Motor right_middle(RIGHT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    
    // Run the right middle wheel for back indexer with direct speed control
    right_middle.move_velocity(speed);
    printf("DEBUG: Right middle motor (back indexer) direct speed: %d\n", speed);
}

void IndexerSystem::runTopIndexer(int speed) {
    // Top indexer is shared between front top and back top scoring
    printf("DEBUG: runTopIndexer() called with speed: %d\n", speed);
    top_indexer.move_velocity(speed);
    printf("DEBUG: Top indexer motor command sent\n");
}

void IndexerSystem::stopTopIndexer() {
    // Stop the top indexer motor
    printf("DEBUG: Stopping top indexer\n");
    top_indexer.move_velocity(0);
    top_indexer.move(0);  // Double-stop to ensure it's off
}

// Testing functions for individual indexer control
void IndexerSystem::testLeftIndexer(int speed) {
    // Ensure PTO is in scorer mode to control left middle wheel
    if (pto_system && pto_system->isDrivetrainMode()) {
        pto_system->setScorerMode();
        pros::delay(100); // Give pneumatics time to actuate
    }
    
    // Create motor object for LEFT middle wheel WITHOUT automatic reversal for direct control
    pros::Motor left_middle(LEFT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    
    // Run left middle wheel for testing with direct speed control
    left_middle.move_velocity(speed);
    
    // LCD call removed to prevent rendering conflicts
}

void IndexerSystem::testRightIndexer(int speed) {
    // Ensure PTO is in scorer mode to control right middle wheel  
    if (pto_system && pto_system->isDrivetrainMode()) {
        pto_system->setScorerMode();
        pros::delay(100); // Give pneumatics time to actuate
    }
    
    // Create motor object for RIGHT middle wheel WITHOUT automatic reversal for direct control
    pros::Motor right_middle(RIGHT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    
    // Run right middle wheel for testing with direct speed control
    right_middle.move_velocity(speed);
    
    // LCD call removed to prevent rendering conflicts
}

void IndexerSystem::stopLeftIndexer() {
    // Stop LEFT middle wheel with direct motor control
    pros::Motor left_middle(LEFT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    left_middle.move_velocity(0);
    left_middle.move(0);  // Double-stop to ensure it's off
    
    // LCD call removed to prevent rendering conflicts
}

void IndexerSystem::stopRightIndexer() {
    // Stop RIGHT middle wheel with direct motor control
    pros::Motor right_middle(RIGHT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    right_middle.move_velocity(0);
    right_middle.move(0);  // Double-stop to ensure it's off
    
    // LCD call removed to prevent rendering conflicts
}
