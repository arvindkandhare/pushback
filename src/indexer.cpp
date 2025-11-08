/**
 * \file indexer.cpp
 *
 * Indexer and scoring system implementation.
 * Manages ball intake, indexing, and scoring for both front and back directions.
 */

#include "indexer.h"
#include <cstdio>
#include <cstring>

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
      score_from_top_storage(false),
      front_flap_open(false),  // Start with flap closed (default state)
      last_collection_button(false),
      last_mid_goal_button(false),
      last_low_goal_button(false),
      last_top_goal_button(false),
      last_front_execute_button(false),
      last_back_execute_button(false),
      last_storage_toggle_button(false),
      last_front_flap_toggle_button(false),
      last_display_update(0),
      force_display_update(true) {
    
    // Set motor brake modes for precise control
    input_motor.set_brake_mode(DRIVETRAIN_BRAKE_MODE);
    top_indexer.set_brake_mode(DRIVETRAIN_BRAKE_MODE);
    
    // Initialize display buffers
    strcpy(last_displayed_line0, "");
    strcpy(last_displayed_line1, ""); 
    strcpy(last_displayed_line2, "");
    
    // Ensure all motors start stopped
    stopAll();
}

void IndexerSystem::setCollectionMode() {
    current_mode = ScoringMode::COLLECTION;
    
    // Debug output to console only
    printf("DEBUG: Set COLLECTION mode\n");
}

void IndexerSystem::setMidGoalMode() {
    current_mode = ScoringMode::MID_GOAL;
    
    // Debug output to console only
    printf("DEBUG: Set MID GOAL mode\n");
}

void IndexerSystem::setLowGoalMode() {
    current_mode = ScoringMode::LOW_GOAL;
    
    // Debug output
    printf("DEBUG: Set LOW GOAL mode\n");
}

void IndexerSystem::setTopGoalMode() {
    current_mode = ScoringMode::TOP_GOAL;
    
    // Debug output
    printf("DEBUG: Set TOP GOAL mode\n");
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
    
    // Stop any currently running sequence (allows interruption)
    if (scoring_active) {
        printf("DEBUG: Interrupting previous sequence (Direction: %s) to start FRONT\n", getDirectionString());
        stopAll();
        // Small delay to ensure motors stop before starting new sequence
        pros::delay(50);
    }
    
    // Set last direction for tracking
    last_direction = ExecutionDirection::FRONT;
    
    // Control front flap only for specific modes
    if (current_mode == ScoringMode::TOP_GOAL) {
        // IMPORTANT: Open front flap for front top goal scoring
        openFrontFlap();
        pros::delay(50); // Give pneumatics time to actuate
    } else if (current_mode == ScoringMode::COLLECTION) {
        // Close front flap for collection to pull balls back
        closeFrontFlap();
        pros::delay(50); // Give pneumatics time to actuate
    }
    // For MID_GOAL and LOW_GOAL: don't change flap status
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
            if (score_from_top_storage) {
                printf("DEBUG: FRONT Collection (STORAGE) - Moving balls from storage toward front\n");
                runLeftIndexer(LEFT_INDEXER_STORAGE_TO_FRONT_SPEED); // Help move balls from storage toward front
                runTopIndexer(TOP_INDEXER_STORAGE_TO_FRONT_SPEED);    // Move balls toward front goal from storage
                runRightIndexer(RIGHT_INDEXER_COLLECTION_SPEED); // Normal collection
            } else {
                printf("DEBUG: FRONT Collection - Left middle motor: %d\n", LEFT_INDEXER_FRONT_COLLECTION_SPEED);
                runLeftIndexer(LEFT_INDEXER_FRONT_COLLECTION_SPEED); // Direct speed for front collection
                runRightIndexer(RIGHT_INDEXER_COLLECTION_SPEED); // Direct speed for back collection
                runTopIndexer(TOP_INDEXER_FRONT_SPEED);  // Direct speed for top indexer front
            }
            startInput(); // Start intake motor for collection
            // LCD call removed to prevent rendering conflicts
            break;
            
        case ScoringMode::MID_GOAL:
            if (score_from_top_storage) {
                printf("DEBUG: FRONT Mid Goal (STORAGE) - Moving balls from storage toward front\n");
                runLeftIndexer(LEFT_INDEXER_FRONT_COLLECTION_SPEED);     // Move balls back from storage
                runTopIndexer(TOP_INDEXER_STORAGE_TO_FRONT_SPEED);        // Move balls toward front goal from storage
            } else {
                printf("DEBUG: FRONT Mid Goal - Left middle motor: %d\n", LEFT_INDEXER_FRONT_MID_GOAL_SPEED);
                runLeftIndexer(LEFT_INDEXER_FRONT_MID_GOAL_SPEED); // Direct speed for front mid goal
            }
            startInput(); // Input motor runs in all scoring modes
            // LCD call removed to prevent rendering conflicts
            break;
            
        case ScoringMode::LOW_GOAL:
            if (score_from_top_storage) {
                printf("DEBUG: FRONT Low Goal (STORAGE) - Moving balls from storage toward front then reverse intake\n");
                runLeftIndexer(LEFT_INDEXER_FRONT_COLLECTION_SPEED); // Move balls back from storage
                runTopIndexer(TOP_INDEXER_STORAGE_TO_FRONT_SPEED);    // Move balls toward front goal from storage
                startInputReverse(); // Run intake motor in reverse for low goal scoring
            } else {
                printf("DEBUG: FRONT Low Goal - Only intake motor reverse: %d\n", INPUT_MOTOR_REVERSE_SPEED);
                startInputReverse(); // Only run intake motor in reverse for low goal
            }
            // LCD call removed to prevent rendering conflicts
            break;
            
        case ScoringMode::TOP_GOAL:
            // For FRONT Top Goal, ignore storage mode - ball is already at front top position
            printf("DEBUG: FRONT Top Goal - Ball already at front top, ignoring storage mode\n");
            printf("DEBUG: FRONT Top Goal - Left middle + top indexer: %d, %d\n", LEFT_INDEXER_FRONT_TOP_GOAL_SPEED, TOP_INDEXER_FRONT_SPEED);
            runLeftIndexer(LEFT_INDEXER_FRONT_TOP_GOAL_SPEED); // Direct speed for front top goal
            runTopIndexer(TOP_INDEXER_FRONT_SPEED);            // Direct speed for top indexer front
            runRightIndexer(RIGHT_INDEXER_TOP_GOAL_HELPER_SPEED); // Slower back indexer to help feed balls smoothly
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
        if (score_from_top_storage) {
            master.print(1, 0, "STORAGE FRONT %s", getModeString());
        } else {
            master.print(1, 0, "FRONT %s", getModeString());
        }
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
    
    // Stop any currently running sequence (allows interruption)
    if (scoring_active) {
        printf("DEBUG: Interrupting previous sequence (Direction: %s) to start BACK\n", getDirectionString());
        stopAll();
        // Small delay to ensure motors stop before starting new sequence
        pros::delay(50);
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
            if (score_from_top_storage) {
                printf("DEBUG: BACK Collection (STORAGE) - Moving balls from storage toward back\n");
                runLeftIndexer(LEFT_INDEXER_FRONT_COLLECTION_SPEED);     // Move balls back from storage
                runTopIndexer(TOP_INDEXER_STORAGE_TO_BACK_SPEED);        // Move balls toward back goal from storage
                runRightIndexer(RIGHT_INDEXER_COLLECTION_SPEED); // Normal collection
            } else {
                printf("DEBUG: BACK Collection - Right: %d, Left helper: %d\n", RIGHT_INDEXER_COLLECTION_SPEED, LEFT_INDEXER_BACK_COLLECTION_SPEED);
                runRightIndexer(RIGHT_INDEXER_COLLECTION_SPEED); // Direct speed for back collection
                runLeftIndexer(LEFT_INDEXER_BACK_COLLECTION_SPEED); // Left motor helps bring ball upwards
            }
            startInput(); // Start intake motor for collection
            // LCD call removed to prevent rendering conflicts
            break;
            
        case ScoringMode::MID_GOAL:
            if (score_from_top_storage) {
                printf("DEBUG: BACK Mid Goal (STORAGE) - Moving balls from storage toward back\n");
                runLeftIndexer(LEFT_INDEXER_FRONT_COLLECTION_SPEED);   // Move balls back from storage
                runTopIndexer(TOP_INDEXER_STORAGE_TO_BACK_SPEED);      // Move balls toward back goal from storage
                runRightIndexer(RIGHT_INDEXER_MID_GOAL_SPEED); // Back mid goal scoring
            } else {
                printf("DEBUG: BACK Mid Goal - Right: %d, Left helper: %d\n", RIGHT_INDEXER_MID_GOAL_SPEED, LEFT_INDEXER_BACK_MID_GOAL_SPEED);
                runRightIndexer(RIGHT_INDEXER_MID_GOAL_SPEED); // Direct speed for back mid goal
                runLeftIndexer(LEFT_INDEXER_BACK_MID_GOAL_SPEED); // Left motor helps bring ball upwards
            }
            startInput(); // Input motor runs in all scoring modes
            // LCD call removed to prevent rendering conflicts
            break;
            
        case ScoringMode::LOW_GOAL:
            if (score_from_top_storage) {
                printf("DEBUG: BACK Low Goal (STORAGE) - Moving balls from storage toward back then reverse intake\n");
                runLeftIndexer(LEFT_INDEXER_FRONT_COLLECTION_SPEED); // Move balls back from storage
                runTopIndexer(TOP_INDEXER_STORAGE_TO_BACK_SPEED);    // Move balls toward back goal from storage
                startInputReverse(); // Run intake motor in reverse for low goal scoring
            } else {
                printf("DEBUG: BACK Low Goal - Only intake motor reverse: %d\n", INPUT_MOTOR_REVERSE_SPEED);
                startInputReverse(); // Only run intake motor in reverse for low goal
            }
            // LCD call removed to prevent rendering conflicts
            break;
            
        case ScoringMode::TOP_GOAL:
            if (score_from_top_storage) {
                printf("DEBUG: BACK Top Goal (STORAGE) - Front toward back + Top toward back + Back scoring\n");
                runLeftIndexer(LEFT_INDEXER_FRONT_COLLECTION_SPEED);   // Front roller toward back (Option B)
                runTopIndexer(TOP_INDEXER_STORAGE_TO_BACK_SPEED);      // Top roller toward back goal
                runRightIndexer(RIGHT_INDEXER_TOP_GOAL_SPEED); // Back roller to back top goal (full speed for scoring)
            } else {
                printf("DEBUG: BACK Top Goal - Right: %d (helper), Top: %d, Left helper: %d\n", RIGHT_INDEXER_TOP_GOAL_HELPER_SPEED, TOP_INDEXER_BACK_SPEED, LEFT_INDEXER_BACK_TOP_GOAL_SPEED);
                runRightIndexer(RIGHT_INDEXER_TOP_GOAL_HELPER_SPEED); // Slower speed to help feed balls smoothly
                runTopIndexer(TOP_INDEXER_BACK_SPEED); // Direct speed for top indexer back
                runLeftIndexer(LEFT_INDEXER_BACK_TOP_GOAL_SPEED); // Left motor helps bring ball upwards
            }
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
        if (score_from_top_storage) {
            master.print(1, 0, "STORAGE BACK %s", getModeString());
        } else {
            master.print(1, 0, "BACK %s", getModeString());
        }
    }
}

void IndexerSystem::openFrontFlap() {
    front_flap.set_value(FRONT_FLAP_OPEN);
    front_flap_open = true;
    printf("DEBUG: Front flap OPENED for scoring\n");
}

void IndexerSystem::closeFrontFlap() {
    front_flap.set_value(FRONT_FLAP_CLOSED);
    front_flap_open = false;
    printf("DEBUG: Front flap CLOSED to hold balls\n");
    // LCD call removed to prevent rendering conflicts
}

void IndexerSystem::toggleFrontFlap() {
    // Check current tracked state and toggle
    if (front_flap_open) {
        closeFrontFlap();
        printf("DEBUG: Manual front flap toggle - CLOSED\n");
    } else {
        openFrontFlap();
        printf("DEBUG: Manual front flap toggle - OPENED\n");
    }
}

void IndexerSystem::startInput() {
    if (!input_motor_active) {
        printf("DEBUG: Starting input motor at %d RPM\n", INPUT_MOTOR_SPEED);
        input_motor.move_velocity(INPUT_MOTOR_SPEED);
        printf("DEBUG: Input motor velocity control: %d RPM\n", INPUT_MOTOR_SPEED);
        input_motor_active = true;
        input_start_time = pros::millis();
        
        printf("DEBUG: Input motor started successfully\n");
    } else {
        printf("DEBUG: Input motor already active\n");
    }
}

void IndexerSystem::startInputReverse() {
    if (!input_motor_active) {
        printf("DEBUG: Starting input motor in REVERSE at %d RPM\n", INPUT_MOTOR_REVERSE_SPEED);
        input_motor.move_velocity(INPUT_MOTOR_REVERSE_SPEED);
        printf("DEBUG: Input motor reverse velocity control: %d RPM\n", INPUT_MOTOR_REVERSE_SPEED);
        input_motor_active = true;
        input_start_time = pros::millis();
        
        printf("DEBUG: Input motor reverse started successfully\n");
    } else {
        printf("DEBUG: Input motor already active\n");
    }
}

void IndexerSystem::startIntakeAndStorage() {
    printf("DEBUG: startIntakeAndStorage() - Face button pressed! Starting intake and moving to storage\n");
    
    // Stop any current scoring operations first
    if (scoring_active) {
        printf("DEBUG: Stopping current operation to start intake and storage\n");
        stopAll();
        pros::delay(100);  // Brief delay to ensure motors stop
    }
    
    // 1. Verify PTO is ready for storage operations
    if (!verifyPTOForStorage()) {
        printf("ERROR: Cannot start storage operation - PTO not ready\n");
        pros::Controller master(pros::E_CONTROLLER_MASTER);
        if (master.is_connected()) {
            master.print(1, 0, "PTO ERROR");
            master.rumble("---");
        }
        return;
    }
    
    // 2. Start intake motor for ball collection
    startInput();
    printf("DEBUG: ✅ Intake motor started\n");
    
    // 3. Close front flap to hold balls for storage (ball containment)
    closeFrontFlap();
    printf("DEBUG: ✅ Front flap closed for storage\n");
    
    // 4. Move balls to top storage position
    // Use moderate speed to move balls up to storage without jamming
    runTopIndexer(60);  // Move balls to storage area (moderate speed)
    printf("DEBUG: ✅ Top indexer moving balls to storage\n");
    
    // 5. Start front indexer to help move balls from intake to storage
    // Use slow speed to avoid jamming and allow smooth ball flow
    runLeftIndexer(LEFT_INDEXER_FRONT_COLLECTION_SPEED / 2);  // Run at half speed (-40) for smooth flow
    printf("DEBUG: ✅ Front indexer moving balls slowly toward storage\n");
    
    // 6. Start right indexer (back indexer) to help feed balls from back
    // Use slow speed to support ball movement toward storage
    runRightIndexer(RIGHT_INDEXER_TOP_GOAL_HELPER_SPEED);  // Run at slow helper speed (-50) 
    printf("DEBUG: ✅ Right indexer (back indexer) helping feed balls slowly\n");
    
    // 7. Set system state flags
    scoring_active = true;
    scoring_start_time = pros::millis();
    last_direction = ExecutionDirection::STORAGE;  // Mark as storage operation
    
    printf("DEBUG: 🎯 Intake and storage sequence started - balls will move to top storage\n");
    
    // 8. Controller feedback
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (master.is_connected()) {
        master.print(1, 0, "STORING: %s", getModeString());
    }
}

void IndexerSystem::stopInput() {
    if (input_motor_active) {
        input_motor.move_velocity(0);  // Stop using velocity control
        input_motor_active = false;
    }
}

void IndexerSystem::stopAll() {
    printf("DEBUG: stopAll() called - resetting all motors and state\n");
    
    // Store previous state for feedback
    bool was_scoring = scoring_active;
    ExecutionDirection previous_direction = last_direction;
    
    // Stop all motors explicitly using velocity control
    input_motor.move_velocity(0);    // Stop velocity control
    input_motor.move_velocity(0);    // Double-stop to ensure it's off
    
    stopLeftIndexer();   // Stop left middle motor (front)
    stopRightIndexer();  // Stop right middle motor (back)
    stopTopIndexer();    // Stop top indexer motor
    
    // IMPORTANT: Close front flap when stopping to hold balls
    closeFrontFlap();
    
    // Reset state completely to ensure system doesn't get stuck
    scoring_active = false;
    input_motor_active = false;
    last_direction = ExecutionDirection::NONE;  // Reset direction to prevent confusion
    
    // Provide feedback about what was stopped
    if (was_scoring) {
        printf("DEBUG: Successfully stopped %s execution flow\n", 
               previous_direction == ExecutionDirection::FRONT ? "FRONT" : 
               previous_direction == ExecutionDirection::BACK ? "BACK" : "UNKNOWN");
    }
    
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
    
    // Get current button states for NEW control scheme
    bool current_collection_button = controller.get_digital(COLLECTION_MODE_BUTTON);     // Y
    bool current_mid_goal_button = controller.get_digital(MID_GOAL_BUTTON);             // A
    bool current_low_goal_button = controller.get_digital(LOW_GOAL_BUTTON);             // B
    bool current_top_goal_button = controller.get_digital(TOP_GOAL_BUTTON);             // X
    bool current_front_execute_button = controller.get_digital(FRONT_EXECUTE_BUTTON);   // R2
    bool current_back_execute_button = controller.get_digital(BACK_EXECUTE_BUTTON);     // R1
    bool current_storage_toggle_button = controller.get_digital(STORAGE_TOGGLE_BUTTON); // LEFT
    bool current_front_flap_toggle_button = controller.get_digital(FRONT_FLAP_TOGGLE_BUTTON); // RIGHT
    
    // Debug: Print button states when any button is pressed
    if (current_collection_button || current_mid_goal_button || current_low_goal_button || 
        current_top_goal_button || current_front_execute_button || current_back_execute_button ||
        current_storage_toggle_button || current_front_flap_toggle_button) {
        printf("DEBUG: Buttons - Y:%d A:%d B:%d X:%d R2:%d R1:%d LEFT:%d RIGHT:%d\n", 
               current_collection_button, current_mid_goal_button, current_low_goal_button,
               current_top_goal_button, current_front_execute_button, current_back_execute_button,
               current_storage_toggle_button, current_front_flap_toggle_button);
    }
    
    // NEW LOGIC: Face buttons (Y, A, B, X) start intake and storage + remember mode WITH TOGGLE
    if (current_collection_button && !last_collection_button) {
        // TOGGLE LOGIC: Check if we're already in collection storage mode
        if (scoring_active && current_mode == ScoringMode::COLLECTION && last_direction == ExecutionDirection::STORAGE) {
            printf("DEBUG: Y pressed again - STOPPING collection storage operation!\n");
            stopAll();
            controller.rumble("--");  // Double dash for stop
            controller.print(1, 0, "STOPPED");
        } else {
            printf("DEBUG: Y (COLLECTION) button pressed - Start intake and storage!\n");
            setCollectionMode();
            startIntakeAndStorage();
            controller.rumble(".");
            controller.print(1, 0, "COLLECTION STORAGE");
        }
        force_display_update = true;
    }
    
    if (current_mid_goal_button && !last_mid_goal_button) {
        // TOGGLE LOGIC: Check if we're already in mid goal storage mode
        if (scoring_active && current_mode == ScoringMode::MID_GOAL && last_direction == ExecutionDirection::STORAGE) {
            printf("DEBUG: A pressed again - STOPPING mid goal storage operation!\n");
            stopAll();
            controller.rumble("--");  // Double dash for stop
            controller.print(1, 0, "STOPPED");
        } else {
            printf("DEBUG: A (MID GOAL) button pressed - Start intake and storage!\n");
            setMidGoalMode();
            startIntakeAndStorage();
            controller.rumble(".");
            controller.print(1, 0, "MID GOAL STORAGE");
        }
        force_display_update = true;
    }
    
    if (current_low_goal_button && !last_low_goal_button) {
        // TOGGLE LOGIC: Check if we're already in low goal storage mode
        if (scoring_active && current_mode == ScoringMode::LOW_GOAL && last_direction == ExecutionDirection::STORAGE) {
            printf("DEBUG: B pressed again - STOPPING low goal storage operation!\n");
            stopAll();
            controller.rumble("--");  // Double dash for stop
            controller.print(1, 0, "STOPPED");
        } else {
            printf("DEBUG: B (LOW GOAL) button pressed - Start intake and storage!\n");
            setLowGoalMode();
            startIntakeAndStorage();
            controller.rumble(".");
            controller.print(1, 0, "LOW GOAL STORAGE");
        }
        force_display_update = true;
    }
    
    if (current_top_goal_button && !last_top_goal_button) {
        // TOGGLE LOGIC: Check if we're already in top goal storage mode
        if (scoring_active && current_mode == ScoringMode::TOP_GOAL && last_direction == ExecutionDirection::STORAGE) {
            printf("DEBUG: X pressed again - STOPPING top goal storage operation!\n");
            stopAll();
            controller.rumble("--");  // Double dash for stop
            controller.print(1, 0, "STOPPED");
        } else {
            printf("DEBUG: X (TOP GOAL) button pressed - Start intake and storage!\n");
            setTopGoalMode();
            startIntakeAndStorage();
            controller.rumble(".");
            controller.print(1, 0, "TOP GOAL STORAGE");
        }
        force_display_update = true;
    }
    
    // NEW LOGIC: R1/R2 buttons - Behavior depends on remembered mode
    if (current_front_execute_button && !last_front_execute_button) {
        printf("DEBUG: R2 (FRONT) button pressed!\n");
        
        if (current_mode == ScoringMode::NONE) {
            printf("DEBUG: No mode selected for FRONT - need to press Y/A/B/X first\n");
            controller.print(1, 0, "Press Y/A/B/X first");
            controller.rumble("---");
        } else if (current_mode == ScoringMode::COLLECTION) {
            // Y MODE: PTO DOWN + intake pushes FORWARD
            printf("DEBUG: R2 with Y mode - PTO DOWN + intake pushes FORWARD!\n");
            
            // Stop any current operation
            if (scoring_active) {
                stopAll();
                pros::delay(100);
            }
            
            // Put PTO DOWN (drivetrain mode)
            if (pto_system && pto_system->isScorerMode()) {
                printf("DEBUG: ✅ Putting PTO DOWN for drivetrain mode\n");
                pto_system->setDrivetrainMode();
                pros::delay(300);  // Allow pneumatics time to actuate
            }
            
            // Use intake motor to push FORWARD
            printf("DEBUG: ✅ Intake motor pushing FORWARD\n");
            startInput();  // Normal intake direction for forward push
            
            // Set state
            scoring_active = true;
            scoring_start_time = pros::millis();
            last_direction = ExecutionDirection::FRONT;
            
            controller.rumble("..");
            controller.print(1, 0, "PUSH FORWARD");
            
        } else {
            // A/B/X MODES: Use indexer for scoring with toggle functionality
            if (scoring_active && last_direction == ExecutionDirection::FRONT) {
                printf("DEBUG: R2 pressed again - TOGGLE back to storage mode (keep mode: %s)\n", getModeString());
                
                // Stop current operation and go back to storage
                stopAll();
                pros::delay(100);
                
                // Go back to storage mode
                startIntakeAndStorage();
                controller.rumble(".-");
                
            } else {
                printf("DEBUG: R2 first press - FRONT scoring with mode: %s\n", getModeString());
                
                // Stop any current operation
                if (scoring_active) {
                    stopAll();
                    pros::delay(100);
                }
                
                last_direction = ExecutionDirection::FRONT;
                executeFront();  // Use indexer system for scoring
                controller.rumble("..");
            }
        }
        force_display_update = true;
    }
    
    if (current_back_execute_button && !last_back_execute_button) {
        printf("DEBUG: R1 (BACK) button pressed!\n");
        
        if (current_mode == ScoringMode::NONE) {
            printf("DEBUG: No mode selected for BACK - need to press Y/A/B/X first\n");
            controller.print(1, 0, "Press Y/A/B/X first");
            controller.rumble("---");
        } else if (current_mode == ScoringMode::COLLECTION) {
            // Y MODE: PTO DOWN + intake pushes BACKWARD
            printf("DEBUG: R1 with Y mode - PTO DOWN + intake pushes BACKWARD!\n");
            
            // Stop any current operation
            if (scoring_active) {
                stopAll();
                pros::delay(100);
            }
            
            // Put PTO DOWN (drivetrain mode)
            if (pto_system && pto_system->isScorerMode()) {
                printf("DEBUG: ✅ Putting PTO DOWN for drivetrain mode\n");
                pto_system->setDrivetrainMode();
                pros::delay(300);  // Allow pneumatics time to actuate
            }
            
            // Use intake motor to push BACKWARD
            printf("DEBUG: ✅ Intake motor pushing BACKWARD\n");
            startInputReverse();  // Reverse intake direction for backward push
            
            // Set state
            scoring_active = true;
            scoring_start_time = pros::millis();
            last_direction = ExecutionDirection::BACK;
            
            controller.rumble("..");
            controller.print(1, 0, "PUSH BACKWARD");
            
        } else {
            // A/B/X MODES: Use indexer for scoring with toggle functionality
            if (scoring_active && last_direction == ExecutionDirection::BACK) {
                printf("DEBUG: R1 pressed again - TOGGLE back to storage mode (keep mode: %s)\n", getModeString());
                
                // Stop current operation and go back to storage
                stopAll();
                pros::delay(100);
                
                // Go back to storage mode
                startIntakeAndStorage();
                controller.rumble(".-");
                
            } else {
                printf("DEBUG: R1 first press - BACK scoring with mode: %s\n", getModeString());
                
                // Stop any current operation
                if (scoring_active) {
                    stopAll();
                    pros::delay(100);
                }
                
                last_direction = ExecutionDirection::BACK;
                executeBack();  // Use indexer system for scoring
                controller.rumble("..");
            }
        }
        force_display_update = true;
    }
    
    // Keep existing functionality for storage toggle and front flap
    if (current_storage_toggle_button && !last_storage_toggle_button) {
        printf("DEBUG: LEFT (STORAGE TOGGLE) button pressed!\n");
        toggleStorageMode();
        force_display_update = true;
    }
    
    if (current_front_flap_toggle_button && !last_front_flap_toggle_button) {
        printf("DEBUG: RIGHT (FRONT FLAP TOGGLE) button pressed!\n");
        toggleFrontFlap();
        controller.rumble("...");
        force_display_update = true;
    }
    
    // Keep existing timeout mechanisms
    if (scoring_active) {
        // Timeout for low goal mode
        if (current_mode == ScoringMode::LOW_GOAL && (pros::millis() - scoring_start_time > 3000)) {
            printf("DEBUG: Low goal mode timeout - automatically stopping (was %s direction)\n", getDirectionString());
            stopAll();
            
            if (controller.is_connected()) {
                controller.print(2, 0, "LOW TIMEOUT");
                controller.rumble("...");
            }
        }
        
        // Different timeouts for different operations
        uint32_t timeout_duration;
        if (last_direction == ExecutionDirection::STORAGE) {
            timeout_duration = 8000;  // 8 seconds for storage operations
        } else if (current_mode == ScoringMode::COLLECTION && 
                  (last_direction == ExecutionDirection::FRONT || last_direction == ExecutionDirection::BACK)) {
            timeout_duration = 3000;  // 3 seconds for Y mode push operations
        } else {
            timeout_duration = 5000;  // 5 seconds for normal scoring operations
        }
        
        if (pros::millis() - scoring_start_time > timeout_duration) {
            printf("DEBUG: Timeout - force stopping %s operations after %d seconds\n", 
                   getDirectionString(), timeout_duration / 1000);
            stopAll();
            
            if (controller.is_connected()) {
                controller.print(2, 0, "TIMEOUT STOP");
                controller.rumble("---");
            }
        }
    }
    
    // Update last button states for next iteration
    last_collection_button = current_collection_button;
    last_mid_goal_button = current_mid_goal_button;
    last_low_goal_button = current_low_goal_button;
    last_top_goal_button = current_top_goal_button;
    last_front_execute_button = current_front_execute_button;
    last_back_execute_button = current_back_execute_button;
    last_storage_toggle_button = current_storage_toggle_button;
    last_front_flap_toggle_button = current_front_flap_toggle_button;
    
    // Update controller display with current status
    updateControllerDisplay(controller, force_display_update);
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

bool IndexerSystem::canInterruptFlow() const {
    // Always allow interruption - this ensures responsive control
    // The system will handle safe motor transitions
    return true;
}

const char* IndexerSystem::getFlowStatus() const {
    static char status_buffer[100];
    
    if (!scoring_active) {
        snprintf(status_buffer, sizeof(status_buffer), "IDLE - Mode: %s", getModeString());
    } else {
        uint32_t elapsed_time = pros::millis() - scoring_start_time;
        snprintf(status_buffer, sizeof(status_buffer), "ACTIVE - %s %s (%dms)", 
                getDirectionString(), getModeString(), elapsed_time);
    }
    
    return status_buffer;
}

char IndexerSystem::getModeChar() const {
    switch (current_mode) {
        case ScoringMode::COLLECTION:  return 'C';
        case ScoringMode::MID_GOAL:    return 'M';
        case ScoringMode::LOW_GOAL:    return 'L';
        case ScoringMode::TOP_GOAL:    return 'T';
        case ScoringMode::NONE:        return '-';
        default: return '?';
    }
}

char IndexerSystem::getDirectionChar() const {
    switch (last_direction) {
        case ExecutionDirection::FRONT: return 'F';
        case ExecutionDirection::BACK:  return 'B';
        case ExecutionDirection::NONE:  return '-';
        default: return '?';
    }
}

char IndexerSystem::getStatusIcon() const {
    if (!scoring_active) {
        return (current_mode == ScoringMode::NONE) ? 'X' : 'O';  // No mode or Ready
    } else {
        return '>';  // Active/Running
    }
}

void IndexerSystem::updateControllerDisplay(pros::Controller& controller, bool force_update) {
    if (!controller.is_connected()) {
        return;
    }
    
    uint32_t current_time = pros::millis();
    
    // Update every 200ms unless forced
    if (!force_update && (current_time - last_display_update < 200)) {
        return;
    }
    
    char line0[17], line1[17], line2[17];
    
    // LINE 0: Mode buttons + Storage + Current Mode Indicator
    // Format: "C●M○L○T● ST○ →T"
    snprintf(line0, sizeof(line0), "C%c M%c L%c T%c ST%c",
             (current_mode == ScoringMode::COLLECTION) ? '*' : 'o',
             (current_mode == ScoringMode::MID_GOAL) ? '*' : 'o', 
             (current_mode == ScoringMode::LOW_GOAL) ? '*' : 'o',
             (current_mode == ScoringMode::TOP_GOAL) ? '*' : 'o',
             score_from_top_storage ? '*' : 'o');
    
    // LINE 1: Execution buttons + Direction indicator
    // Format: "R2○ R1● →BACK"  
    snprintf(line1, sizeof(line1), "R2%c R1%c %c%c",
             (scoring_active && last_direction == ExecutionDirection::FRONT) ? '*' : 'o',
             (scoring_active && last_direction == ExecutionDirection::BACK) ? '*' : 'o',
             scoring_active ? '>' : '-',
             getDirectionChar());
    
    // LINE 2: Mode name + Runtime + Status
    // Format: "COLLECT 2.1s >"
    if (scoring_active) {
        float runtime = (current_time - scoring_start_time) / 1000.0f;
        snprintf(line2, sizeof(line2), "%s %.1fs %c", 
                getModeString(), runtime, getStatusIcon());
    } else {
        snprintf(line2, sizeof(line2), "%s READY %c", 
                getModeString(), getStatusIcon());
    }
    
    // Only update lines that have changed to reduce flicker
    if (strcmp(line0, last_displayed_line0) != 0 || force_update) {
        controller.print(0, 0, "%s", line0);
        strcpy(last_displayed_line0, line0);
    }
    
    if (strcmp(line1, last_displayed_line1) != 0 || force_update) {
        controller.print(1, 0, "%s", line1);
        strcpy(last_displayed_line1, line1);
    }
    
    if (strcmp(line2, last_displayed_line2) != 0 || force_update) {
        controller.print(2, 0, "%s", line2);
        strcpy(last_displayed_line2, line2);
    }
    
    last_display_update = current_time;
    force_display_update = false;
}

const char* IndexerSystem::getDirectionString() const {
    switch (last_direction) {
        case ExecutionDirection::FRONT:   return "FRONT";
        case ExecutionDirection::BACK:    return "BACK";
        case ExecutionDirection::STORAGE: return "STORAGE";  // NEW
        case ExecutionDirection::NONE:    return "NONE";
        default: return "UNKNOWN";
    }
}

void IndexerSystem::runLeftIndexer(int speed) {
    // Left indexer uses the LEFT middle wheel via PTO for front storage/scoring
    printf("DEBUG: runLeftIndexer() called with speed: %d RPM\n", speed);
    
    // Create motor object for LEFT middle wheel WITHOUT automatic reversal for direct control
    pros::Motor left_middle(LEFT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    
    // Use velocity control (RPM) - maintains full torque at target speed
    left_middle.move_velocity(speed);
    printf("DEBUG: Left middle motor (front indexer) velocity control: %d RPM\n", speed);
}

void IndexerSystem::runRightIndexer(int speed) {
    // Right indexer uses the RIGHT middle wheel via PTO for back scoring
    printf("DEBUG: runRightIndexer() called with speed: %d RPM\n", speed);
    
    // Create motor object for RIGHT middle wheel WITHOUT automatic reversal for direct control
    pros::Motor right_middle(RIGHT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    
    // Use velocity control (RPM) - maintains full torque at target speed
    right_middle.move_velocity(speed);
    printf("DEBUG: Right middle motor (back indexer) velocity control: %d RPM\n", speed);
}

void IndexerSystem::runTopIndexer(int speed) {
    // Top indexer is shared between front top and back top scoring
    printf("DEBUG: runTopIndexer() called with speed: %d RPM\n", speed);
    
    // Use velocity control (RPM) - maintains full torque at target speed
    top_indexer.move_velocity(speed);
    printf("DEBUG: Top indexer velocity control: %d RPM\n", speed);
}

void IndexerSystem::stopTopIndexer() {
    // Stop the top indexer motor
    printf("DEBUG: Stopping top indexer\n");
    top_indexer.move_velocity(0);  // Stop using velocity control
}


void IndexerSystem::stopLeftIndexer() {
    // Stop LEFT middle wheel with direct motor control
    pros::Motor left_middle(LEFT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    left_middle.move_velocity(0);  // Stop using velocity control
}

void IndexerSystem::stopRightIndexer() {
    // Stop RIGHT middle wheel with direct motor control
    pros::Motor right_middle(RIGHT_MIDDLE_MOTOR_PORT, DRIVETRAIN_GEARSET);
    right_middle.move_velocity(0);  // Stop using velocity control
}

void IndexerSystem::toggleStorageMode() {
    score_from_top_storage = !score_from_top_storage;
    
    printf("DEBUG: Storage mode toggled to: %s\n", score_from_top_storage ? "ACTIVE" : "INACTIVE");
}

bool IndexerSystem::isStorageModeActive() const {
    return score_from_top_storage;
}

bool IndexerSystem::verifyPTOForStorage() {
    if (!pto_system) {
        printf("WARNING: PTO system not available for storage operations\n");
        return false;
    }
    
    if (pto_system->isDrivetrainMode()) {
        printf("DEBUG: PTO in drivetrain mode, switching to scorer mode for storage...\n");
        pto_system->setScorerMode();
        pros::delay(200); // Wait for pneumatic actuation
        
        // Verify the switch worked (if PTO has state verification)
        if (pto_system->isDrivetrainMode()) {
            printf("ERROR: PTO failed to switch to scorer mode!\n");
            return false;
        }
        printf("DEBUG: ✅ PTO successfully switched to scorer mode\n");
    }
    
    return true;
}
