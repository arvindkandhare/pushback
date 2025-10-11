/**
 * \file indexer.cpp
 *
 * Indexer and scoring system implementation.
 * Manages ball intake, indexing, and scoring for both front and back directions.
 */

#include "indexer.h"

IndexerSystem::IndexerSystem(PTO* pto) 
    : input_motor(INPUT_MOTOR_PORT, DRIVETRAIN_GEARSET),
      front_indexer(FRONT_INDEXER_PORT, DRIVETRAIN_GEARSET),
      pto_system(pto),
      current_direction(ScoringDirection::NONE),
      current_level(ScoringLevel::NONE),
      scoring_active(false),
      scoring_start_time(0),
      input_start_time(0),
      input_motor_active(false),
      last_front_button(false),
      last_back_button(false),
      last_long_goal_button(false),
      last_mid_goal_button(false),
      last_execute_button(false),
      last_input_button(false) {
    
    // Set motor brake modes for precise control
    input_motor.set_brake_mode(DRIVETRAIN_BRAKE_MODE);
    front_indexer.set_brake_mode(DRIVETRAIN_BRAKE_MODE);
    
    // Ensure all motors start stopped
    stopAll();
}

void IndexerSystem::setFrontScoring() {
    current_direction = ScoringDirection::FRONT;
    
    // Debug output to multiple places
    pros::lcd::print(1, "Scoring: FRONT %s", getLevelString());
    printf("DEBUG: Set FRONT scoring direction\n");
    
    // Send to controller if available
    if (pto_system) {
        pros::Controller master(pros::E_CONTROLLER_MASTER);
        if (master.is_connected()) {
            master.print(0, 0, "FRONT %s", getLevelString());
        }
    }
}

void IndexerSystem::setBackScoring() {
    current_direction = ScoringDirection::BACK;
    
    // Debug output to multiple places
    pros::lcd::print(1, "Scoring: BACK %s", getLevelString());
    printf("DEBUG: Set BACK scoring direction\n");
    
    // Send to controller if available
    if (pto_system) {
        pros::Controller master(pros::E_CONTROLLER_MASTER);
        if (master.is_connected()) {
            master.print(0, 0, "BACK %s", getLevelString());
        }
    }
}

void IndexerSystem::setLongGoal() {
    current_level = ScoringLevel::LONG_GOAL;
    
    // Debug output to multiple places
    pros::lcd::print(1, "Scoring: %s LONG GOAL", getDirectionString());
    printf("DEBUG: Set LONG GOAL level\n");
    
    // Send to controller if available
    if (pto_system) {
        pros::Controller master(pros::E_CONTROLLER_MASTER);
        if (master.is_connected()) {
            master.print(0, 0, "%s LONG", getDirectionString());
        }
    }
}

void IndexerSystem::setMidGoal() {
    current_level = ScoringLevel::MID_GOAL;
    
    // Debug output to multiple places
    pros::lcd::print(1, "Scoring: %s MID GOAL", getDirectionString());
    printf("DEBUG: Set MID GOAL level\n");
    
    // Send to controller if available
    if (pto_system) {
        pros::Controller master(pros::E_CONTROLLER_MASTER);
        if (master.is_connected()) {
            master.print(0, 0, "%s MID", getDirectionString());
        }
    }
}

void IndexerSystem::executeScoring() {
    printf("DEBUG: executeScoring() called\n");
    
    // Can't score without both direction and level selected
    if (current_direction == ScoringDirection::NONE || current_level == ScoringLevel::NONE) {
        printf("DEBUG: Missing direction or level - Direction: %d, Level: %d\n", 
               (int)current_direction, (int)current_level);
        pros::lcd::print(2, "Select direction & level first!");
        if (pto_system) {
            pros::Controller master(pros::E_CONTROLLER_MASTER);
            if (master.is_connected()) {
                master.print(1, 0, "Need Dir+Level");
            }
        }
        return;
    }
    
    printf("DEBUG: Scoring with Direction: %d, Level: %d\n", 
           (int)current_direction, (int)current_level);
    
    // Stop any currently running scoring sequence
    if (scoring_active) {
        printf("DEBUG: Stopping previous scoring sequence\n");
        stopAll();
    }
    
    // Determine motor speed based on scoring level
    int motor_speed;
    int direction_multiplier;
    
    if (current_level == ScoringLevel::LONG_GOAL) {
        motor_speed = INDEXER_SPEED_TOP_SCORING;  // 150
    } else {
        motor_speed = INDEXER_SPEED_BOTTOM_SCORING;  // 100
    }
    
    printf("DEBUG: Motor speed determined: %d RPM\n", motor_speed);
    
    // Execute scoring based on direction
    if (current_direction == ScoringDirection::FRONT) {
        printf("DEBUG: Executing FRONT scoring\n");
        
        // Front scoring - use front indexer
        if (current_level == ScoringLevel::LONG_GOAL) {
            direction_multiplier = FRONT_INDEXER_TOP_DIRECTION;  // +1
        } else {
            direction_multiplier = FRONT_INDEXER_BOTTOM_DIRECTION;  // -1
        }
        
        printf("DEBUG: Front indexer - Speed: %d, Direction: %d, Final: %d\n", 
               motor_speed, direction_multiplier, motor_speed * direction_multiplier);
        
        runFrontIndexer(motor_speed * direction_multiplier);
        pros::lcd::print(2, "FRONT scoring active...");
        
        if (pto_system) {
            pros::Controller master(pros::E_CONTROLLER_MASTER);
            if (master.is_connected()) {
                master.print(1, 0, "FRONT ACTIVE");
            }
        }
        
    } else if (current_direction == ScoringDirection::BACK) {
        // Back scoring - use RIGHT side PTO only
        // TODO: When PTO class supports independent control, use:
        // pto_system->setRightScorerMode(); // Only switch right side
        
        // For now, use entire PTO system (will be updated when independent control is implemented)
        if (pto_system->isDrivetrainMode()) {
            pto_system->setScorerMode();
            pros::delay(100); // Give pneumatics time to actuate
        }
        
        if (current_level == ScoringLevel::LONG_GOAL) {
            direction_multiplier = BACK_INDEXER_TOP_DIRECTION;
        } else {
            direction_multiplier = BACK_INDEXER_BOTTOM_DIRECTION;
        }
        
        // Note: runBackIndexer now uses ONLY the right middle wheel
        runBackIndexer(motor_speed * direction_multiplier);
        pros::lcd::print(2, "BACK scoring (RIGHT PTO) active...");
    }
    
    // Start scoring sequence timer
    scoring_active = true;
    scoring_start_time = pros::millis();
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
    
    stopFrontIndexer();
    stopBackIndexer();
    
    // Reset state
    scoring_active = false;
    input_motor_active = false;
    
    pros::lcd::print(2, "All motors STOPPED");
    pros::lcd::print(3, "");
}

ScoringDirection IndexerSystem::getCurrentDirection() const {
    return current_direction;
}

ScoringLevel IndexerSystem::getCurrentLevel() const {
    return current_level;
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
    
    // Get current button states
    bool current_front_button = controller.get_digital(FRONT_SCORING_BUTTON);
    bool current_back_button = controller.get_digital(BACK_SCORING_BUTTON);
    bool current_long_goal_button = controller.get_digital(LONG_GOAL_BUTTON);
    bool current_mid_goal_button = controller.get_digital(MID_GOAL_BUTTON);
    bool current_execute_button = controller.get_digital(EXECUTE_SCORING_BUTTON);
    bool current_input_button = controller.get_digital(INPUT_MOTOR_BUTTON);
    
    // Debug: Print button states when any button is pressed
    if (current_front_button || current_back_button || current_long_goal_button || 
        current_mid_goal_button || current_execute_button || current_input_button) {
        printf("DEBUG: Buttons - L1:%d L2:%d X:%d B:%d A:%d R2:%d\n", 
               current_front_button, current_back_button, current_long_goal_button,
               current_mid_goal_button, current_execute_button, current_input_button);
    }
    
    // Handle direction selection (rising edge detection)
    if (current_front_button && !last_front_button) {
        printf("DEBUG: L1 (FRONT) button pressed!\n");
        setFrontScoring();
        controller.rumble(".");
        if (controller.is_connected()) {
            controller.print(2, 0, "L1 PRESSED");
        }
    }
    
    if (current_back_button && !last_back_button) {
        printf("DEBUG: L2 (BACK) button pressed!\n");
        setBackScoring();
        controller.rumble(".");
        if (controller.is_connected()) {
            controller.print(2, 0, "L2 PRESSED");
        }
    }
    
    // Handle level selection (rising edge detection)
    if (current_long_goal_button && !last_long_goal_button) {
        printf("DEBUG: X (LONG GOAL) button pressed!\n");
        setLongGoal();
        controller.rumble(".");
        if (controller.is_connected()) {
            controller.print(2, 0, "X PRESSED");
        }
    }
    
    if (current_mid_goal_button && !last_mid_goal_button) {
        printf("DEBUG: B (MID GOAL) button pressed!\n");
        setMidGoal();
        controller.rumble(".");
        if (controller.is_connected()) {
            controller.print(2, 0, "B PRESSED");
        }
    }
    
    // Handle scoring execution (rising edge detection)
    if (current_execute_button && !last_execute_button) {
        printf("DEBUG: A (EXECUTE) button pressed!\n");
        executeScoring();
        controller.rumble("..");
        if (controller.is_connected()) {
            controller.print(2, 0, "A PRESSED");
        }
    }
    
    // Handle input motor (hold to run)
    if (current_input_button) {
        if (!input_motor_active) {
            printf("DEBUG: R2 (INPUT) button pressed - starting input motor!\n");
            if (controller.is_connected()) {
                controller.print(2, 0, "R2 ON");
            }
        }
        startInput();
    } else {
        if (input_motor_active) {
            printf("DEBUG: R2 (INPUT) button released - stopping input motor!\n");
            if (controller.is_connected()) {
                controller.print(2, 0, "R2 OFF");
            }
        }
        stopInput();
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
    last_front_button = current_front_button;
    last_back_button = current_back_button;
    last_long_goal_button = current_long_goal_button;
    last_mid_goal_button = current_mid_goal_button;
    last_execute_button = current_execute_button;
    last_input_button = current_input_button;
}

const char* IndexerSystem::getDirectionString() const {
    switch (current_direction) {
        case ScoringDirection::FRONT: return "FRONT";
        case ScoringDirection::BACK:  return "BACK";
        case ScoringDirection::NONE:  return "NONE";
        default: return "UNKNOWN";
    }
}

const char* IndexerSystem::getLevelString() const {
    switch (current_level) {
        case ScoringLevel::LONG_GOAL: return "LONG GOAL";
        case ScoringLevel::MID_GOAL:  return "MID GOAL";
        case ScoringLevel::NONE:      return "NONE";
        default: return "UNKNOWN";
    }
}

void IndexerSystem::runFrontIndexer(int speed) {
    printf("DEBUG: runFrontIndexer() called with speed: %d\n", speed);
    front_indexer.move_velocity(speed);
    printf("DEBUG: Front indexer motor command sent\n");
}

void IndexerSystem::runBackIndexer(int speed) {
    // Back indexer uses ONLY the RIGHT side PTO middle wheel
    // This allows the left side to remain in drive mode for asymmetric operation
    
    // Create motor object for RIGHT middle wheel only with proper PROS v4.2.1 constructor
    pros::Motor right_middle(RIGHT_MOTORS_REVERSED ? -RIGHT_MIDDLE_MOTOR_PORT : RIGHT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    
    // Run only the right middle wheel for back indexer
    right_middle.move_velocity(speed);
}

void IndexerSystem::stopFrontIndexer() {
    front_indexer.move_velocity(0);
    front_indexer.move(0);  // Double-stop to ensure it's off
}

void IndexerSystem::stopBackIndexer() {
    // Stop ONLY the RIGHT side middle wheel (back indexer)
    pros::Motor right_middle(RIGHT_MOTORS_REVERSED ? -RIGHT_MIDDLE_MOTOR_PORT : RIGHT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    
    right_middle.move_velocity(0);
    right_middle.move(0);  // Double-stop to ensure it's off
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
    
    // Create motor object for LEFT middle wheel with proper PROS v4.2.1 constructor
    pros::Motor left_middle(LEFT_MOTORS_REVERSED ? -LEFT_MIDDLE_MOTOR_PORT : LEFT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    
    // Run left middle wheel for testing
    left_middle.move_velocity(speed);
    
    pros::lcd::print(2, "Testing LEFT indexer: %d RPM", speed);
}

void IndexerSystem::testRightIndexer(int speed) {
    // Ensure PTO is in scorer mode to control right middle wheel  
    if (pto_system && pto_system->isDrivetrainMode()) {
        pto_system->setScorerMode();
        pros::delay(100); // Give pneumatics time to actuate
    }
    
    // Create motor object for RIGHT middle wheel with proper PROS v4.2.1 constructor
    pros::Motor right_middle(RIGHT_MOTORS_REVERSED ? -RIGHT_MIDDLE_MOTOR_PORT : RIGHT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    
    // Run right middle wheel for testing
    right_middle.move_velocity(speed);
    
    pros::lcd::print(2, "Testing RIGHT indexer: %d RPM", speed);
}

void IndexerSystem::stopLeftIndexer() {
    // Stop LEFT middle wheel
    pros::Motor left_middle(LEFT_MOTORS_REVERSED ? -LEFT_MIDDLE_MOTOR_PORT : LEFT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    left_middle.move_velocity(0);
    left_middle.move(0);  // Double-stop to ensure it's off
    
    pros::lcd::print(2, "LEFT indexer STOPPED");
}

void IndexerSystem::stopRightIndexer() {
    // Stop RIGHT middle wheel
    pros::Motor right_middle(RIGHT_MOTORS_REVERSED ? -RIGHT_MIDDLE_MOTOR_PORT : RIGHT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    right_middle.move_velocity(0);
    right_middle.move(0);  // Double-stop to ensure it's off
    
    pros::lcd::print(2, "RIGHT indexer STOPPED");
}