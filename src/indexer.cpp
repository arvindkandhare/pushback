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
      last_immediate_button(false),
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
    
    // Debug output to multiple places
    pros::lcd::print(1, "Mode: COLLECTION");
    printf("DEBUG: Set COLLECTION mode\n");
    
    // Send to controller if available
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (master.is_connected()) {
        master.print(0, 0, "COLLECTION");
    }
}

void IndexerSystem::setMidGoalMode() {
    current_mode = ScoringMode::MID_GOAL;
    
    // Debug output
    pros::lcd::print(1, "Mode: MID GOAL");
    printf("DEBUG: Set MID GOAL mode\n");
    
    // Send to controller
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (master.is_connected()) {
        master.print(0, 0, "MID GOAL");
    }
}

void IndexerSystem::setImmediateMode() {
    current_mode = ScoringMode::IMMEDIATE;
    
    // Debug output
    pros::lcd::print(1, "Mode: IMMEDIATE");
    printf("DEBUG: Set IMMEDIATE mode\n");
    
    // Send to controller
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (master.is_connected()) {
        master.print(0, 0, "IMMEDIATE");
    }
}

void IndexerSystem::setTopGoalMode() {
    current_mode = ScoringMode::TOP_GOAL;
    
    // Debug output
    pros::lcd::print(1, "Mode: TOP GOAL");
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
        pros::lcd::print(2, "Select mode first!");
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
    
    // IMPORTANT: Open front flap for scoring
    openFrontFlap();
    pros::delay(100); // Give pneumatics time to actuate
    
    // Ensure PTO is in scorer mode for front indexer (left middle motor)
    if (pto_system && pto_system->isDrivetrainMode()) {
        pto_system->setScorerMode();
        pros::delay(100); // Give pneumatics time to actuate
    }
    
    // Execute based on mode
    int motor_speed;
    
    switch (current_mode) {
        case ScoringMode::COLLECTION:
            motor_speed = INPUT_MOTOR_SPEED; // 120 RPM
            printf("DEBUG: FRONT Collection - Left middle motor: %d\n", motor_speed);
            runLeftIndexer(motor_speed); // Use left middle motor for front collection/storage
            startInput(); // Start intake motor for collection
            pros::lcd::print(2, "FRONT Collection + Intake...");
            break;
            
        case ScoringMode::MID_GOAL:
            motor_speed = INDEXER_SPEED_BOTTOM_SCORING; // 100 RPM
            printf("DEBUG: FRONT Mid Goal - Left middle motor: %d\n", motor_speed);
            runLeftIndexer(motor_speed); // Use left middle motor for front mid scoring
            pros::lcd::print(2, "FRONT Mid Goal active...");
            break;
            
        case ScoringMode::IMMEDIATE:
            motor_speed = INPUT_MOTOR_SPEED; // 120 RPM - immediate from intake
            printf("DEBUG: FRONT Immediate - Left middle motor: %d\n", motor_speed);
            runLeftIndexer(motor_speed); // Use left middle motor for front immediate
            startInput(); // Start intake motor for immediate scoring
            pros::lcd::print(2, "FRONT Immediate + Intake...");
            break;
            
        case ScoringMode::TOP_GOAL:
            motor_speed = INDEXER_SPEED_TOP_SCORING; // 150 RPM
            printf("DEBUG: FRONT Top Goal - Left middle + top indexer: %d\n", motor_speed);
            runLeftIndexer(motor_speed); // Use left middle motor
            runTopIndexer(motor_speed);  // Also use top indexer for top scoring
            pros::lcd::print(2, "FRONT Top Goal active...");
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
        pros::lcd::print(2, "Select mode first!");
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
    
    // Ensure PTO is in scorer mode for back indexer
    if (pto_system && pto_system->isDrivetrainMode()) {
        pto_system->setScorerMode();
        pros::delay(100); // Give pneumatics time to actuate
    }
    
    // Execute based on mode
    int motor_speed;
    
    switch (current_mode) {
        case ScoringMode::COLLECTION:
            motor_speed = INPUT_MOTOR_SPEED; // 120 RPM
            printf("DEBUG: BACK Collection - Right middle motor: %d\n", motor_speed);
            runRightIndexer(motor_speed); // Use right middle motor (no storage, just movement)
            startInput(); // Start intake motor for collection
            pros::lcd::print(2, "BACK Collection + Intake...");
            break;
            
        case ScoringMode::MID_GOAL:
            motor_speed = INDEXER_SPEED_BOTTOM_SCORING; // 100 RPM
            printf("DEBUG: BACK Mid Goal - Right middle motor: %d\n", motor_speed);
            runRightIndexer(motor_speed); // Use right middle motor for back mid scoring
            pros::lcd::print(2, "BACK Mid Goal active...");
            break;
            
        case ScoringMode::IMMEDIATE:
            motor_speed = INPUT_MOTOR_SPEED; // 120 RPM - immediate from intake
            printf("DEBUG: BACK Immediate - Right middle motor: %d\n", motor_speed);
            runRightIndexer(motor_speed); // Use right middle motor for back immediate
            startInput(); // Start intake motor for immediate scoring
            pros::lcd::print(2, "BACK Immediate + Intake...");
            break;
            
        case ScoringMode::TOP_GOAL:
            motor_speed = INDEXER_SPEED_TOP_SCORING; // 150 RPM
            printf("DEBUG: BACK Top Goal - Right middle + top indexer: %d\n", motor_speed);
            runRightIndexer(motor_speed); // Use right middle motor
            runTopIndexer(motor_speed);   // Also use top indexer for top scoring
            pros::lcd::print(2, "BACK Top Goal active...");
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
    pros::lcd::print(3, "Front flap: OPEN");
}

void IndexerSystem::closeFrontFlap() {
    front_flap.set_value(FRONT_FLAP_CLOSED);
    printf("DEBUG: Front flap CLOSED to hold balls\n");
    pros::lcd::print(3, "Front flap: CLOSED");
}

void IndexerSystem::startInput() {
    if (!input_motor_active) {
        printf("DEBUG: Starting input motor at %d RPM\n", INPUT_MOTOR_SPEED);
        input_motor.move_velocity(INPUT_MOTOR_SPEED);
        input_motor_active = true;
        input_start_time = pros::millis();
        
        pros::lcd::print(3, "Input motor: ACTIVE");
        printf("DEBUG: Input motor started successfully\n");
    } else {
        printf("DEBUG: Input motor already active\n");
    }
}

void IndexerSystem::stopInput() {
    if (input_motor_active) {
        input_motor.move_velocity(0);
        input_motor_active = false;
        
        pros::lcd::print(3, "Input motor: STOPPED");
    }
}

void IndexerSystem::stopAll() {
    // Stop all motors explicitly
    input_motor.move_velocity(0);
    input_motor.move(0);  // Double-stop to ensure it's off
    
    stopLeftIndexer();   // Stop left middle motor (front)
    stopRightIndexer();  // Stop right middle motor (back)
    stopTopIndexer();    // Stop top indexer motor
    
    // IMPORTANT: Close front flap when stopping to hold balls
    closeFrontFlap();
    
    // Reset state
    scoring_active = false;
    input_motor_active = false;
    
    pros::lcd::print(2, "All motors STOPPED");
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
    bool current_immediate_button = controller.get_digital(IMMEDIATE_SCORING_BUTTON);   // B
    bool current_top_goal_button = controller.get_digital(TOP_GOAL_BUTTON);             // X
    bool current_front_execute_button = controller.get_digital(FRONT_EXECUTE_BUTTON);   // R2
    bool current_back_execute_button = controller.get_digital(BACK_EXECUTE_BUTTON);     // R1
    
    // Debug: Print button states when any button is pressed
    if (current_collection_button || current_mid_goal_button || current_immediate_button || 
        current_top_goal_button || current_front_execute_button || current_back_execute_button) {
        printf("DEBUG: Buttons - Y:%d A:%d B:%d X:%d R2:%d R1:%d\n", 
               current_collection_button, current_mid_goal_button, current_immediate_button,
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
    
    if (current_immediate_button && !last_immediate_button) {
        printf("DEBUG: B (IMMEDIATE) button pressed!\n");
        setImmediateMode();
        controller.rumble(".");
        if (controller.is_connected()) {
            controller.print(2, 0, "B IMMEDIATE");
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
    
    // Check for automatic timeouts
    if (scoring_active && checkScoringTimeout()) {
        stopAll();
        controller.rumble("---");  // Long rumble to indicate timeout
    }
    
    if (input_motor_active && checkInputTimeout()) {
        stopInput();
        controller.rumble("--");   // Medium rumble to indicate input timeout
    }
    
    // Update last button states for next iteration
    last_collection_button = current_collection_button;
    last_mid_goal_button = current_mid_goal_button;
    last_immediate_button = current_immediate_button;
    last_top_goal_button = current_top_goal_button;
    last_front_execute_button = current_front_execute_button;
    last_back_execute_button = current_back_execute_button;
}

const char* IndexerSystem::getModeString() const {
    switch (current_mode) {
        case ScoringMode::COLLECTION:  return "COLLECTION";
        case ScoringMode::MID_GOAL:    return "MID GOAL";
        case ScoringMode::IMMEDIATE:   return "IMMEDIATE";
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
    
    // Create motor object for LEFT middle wheel
    pros::Motor left_middle(LEFT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    if (LEFT_MOTORS_REVERSED) {
        speed = -speed; // Reverse speed if needed
    }
    
    // Run the left middle wheel for front indexer
    left_middle.move_velocity(speed);
    printf("DEBUG: Left middle motor (front indexer) command sent\n");
}

void IndexerSystem::runRightIndexer(int speed) {
    // Right indexer uses the RIGHT middle wheel via PTO for back scoring
    printf("DEBUG: runRightIndexer() called with speed: %d\n", speed);
    
    // Create motor object for RIGHT middle wheel
    pros::Motor right_middle(RIGHT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    if (RIGHT_MOTORS_REVERSED) {
        speed = -speed; // Reverse speed if needed
    }
    
    // Run the right middle wheel for back indexer
    right_middle.move_velocity(speed);
    printf("DEBUG: Right middle motor (back indexer) command sent\n");
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

bool IndexerSystem::checkScoringTimeout() {
    if (scoring_active) {
        return (pros::millis() - scoring_start_time) > SCORING_SEQUENCE_DURATION;
    }
    return false;
}

bool IndexerSystem::checkInputTimeout() {
    if (input_motor_active) {
        return (pros::millis() - input_start_time) > INPUT_MOTOR_TIMEOUT;
    }
    return false;
}

// Testing functions for individual indexer control
void IndexerSystem::testLeftIndexer(int speed) {
    // Ensure PTO is in scorer mode to control left middle wheel
    if (pto_system && pto_system->isDrivetrainMode()) {
        pto_system->setScorerMode();
        pros::delay(100); // Give pneumatics time to actuate
    }
    
    // Create motor object for LEFT middle wheel
    pros::Motor left_middle(LEFT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    if (LEFT_MOTORS_REVERSED) {
        speed = -speed; // Reverse speed if needed
    }
    
    // Run left middle wheel for testing
    left_middle.move_velocity(speed);
    left_middle.move_velocity(speed);
    
    pros::lcd::print(2, "Testing LEFT indexer: %d RPM", speed);
}

void IndexerSystem::testRightIndexer(int speed) {
    // Ensure PTO is in scorer mode to control right middle wheel  
    if (pto_system && pto_system->isDrivetrainMode()) {
        pto_system->setScorerMode();
        pros::delay(100); // Give pneumatics time to actuate
    }
    
    // Create motor object for RIGHT middle wheel
    pros::Motor right_middle(RIGHT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    if (RIGHT_MOTORS_REVERSED) {
        speed = -speed; // Reverse speed if needed
    }
    
    // Run right middle wheel for testing
    right_middle.move_velocity(speed);
    
    pros::lcd::print(2, "Testing RIGHT indexer: %d RPM", speed);
}

void IndexerSystem::stopLeftIndexer() {
    // Stop LEFT middle wheel
    pros::Motor left_middle(LEFT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    left_middle.move_velocity(0);
    left_middle.move(0);  // Double-stop to ensure it's off
    
    pros::lcd::print(2, "LEFT indexer STOPPED");
}

void IndexerSystem::stopRightIndexer() {
    // Stop RIGHT middle wheel
    pros::Motor right_middle(RIGHT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    right_middle.move_velocity(0);
    right_middle.move(0);  // Double-stop to ensure it's off
    
    pros::lcd::print(2, "RIGHT indexer STOPPED");
}