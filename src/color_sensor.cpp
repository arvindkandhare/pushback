/**
 * \file color_sensor.cpp
 *
 * Implementation of the color sensing and sorting system.
 * Provides real-time ball color detection and automatic ejection
 * of undesired colored balls using the existing indexer system.
 */

#include "color_sensor.h"
#include "indexer.h"
#include <cstring>

// Global instance
ColorSensorSystem* color_sensor_system = nullptr;

// =============================================================================
// CONSTRUCTOR AND DESTRUCTOR
// =============================================================================

ColorSensorSystem::ColorSensorSystem() {
    // Initialize hardware pointers
    sensor1 = nullptr;
    sensor2 = nullptr;
    indexer_system = nullptr;
    
    // Initialize state variables
    current_mode = SortingMode::COLLECT_ALL;
    last_detected_color = BallColor::UNKNOWN;
    last_direction = BallDirection::UNKNOWN;
    sensor1_triggered = false;
    sensor2_triggered = false;
    sensor1_trigger_time = 0;
    sensor2_trigger_time = 0;
    ejection_active = false;
    ejection_start_time = 0;
    ejection_duration = BALL_EJECT_DURATION_MS; // Default ejection duration
    
    // Initialize state preservation structure
    saved_indexer_state.was_scoring_active = false;
    saved_indexer_state.was_input_active = false;
    saved_indexer_state.saved_scoring_mode = 0;      // ScoringMode::COLLECTION as int
    saved_indexer_state.saved_execution_direction = 0; // ExecutionDirection::NONE as int
    saved_indexer_state.valid = false;
    
    // Initialize detection buffers
    for (int i = 0; i < COLOR_CONFIRMATION_COUNT; i++) {
        sensor1_color_buffer[i] = BallColor::NO_BALL;
        sensor2_color_buffer[i] = BallColor::NO_BALL;
    }
    sensor1_buffer_index = 0;
    sensor2_buffer_index = 0;
    
    // Initialize statistics
    red_balls_detected = 0;
    blue_balls_detected = 0;
    balls_ejected = 0;
    false_detections = 0;
    
    // Create hardware objects
    try {
        sensor1 = new pros::Optical(COLOR_SENSOR_1_PORT);
        sensor2 = new pros::Optical(COLOR_SENSOR_2_PORT);
        
        printf("✅ Color sensor system hardware objects created successfully\n");
    } catch (const std::exception& e) {
        printf("❌ Error creating color sensor hardware: %s\n", e.what());
    }
}

ColorSensorSystem::~ColorSensorSystem() {
    // Stop any active ejection sequence
    if (ejection_active) {
        stopEjection();
    }
    
    // Clean up hardware objects
    if (sensor1) {
        delete sensor1;
        sensor1 = nullptr;
    }
    if (sensor2) {
        delete sensor2;
        sensor2 = nullptr;
    }
    
    printf("🔧 Color sensor system cleaned up\n");
}

// =============================================================================
// INITIALIZATION AND SETUP
// =============================================================================

bool ColorSensorSystem::initialize(IndexerSystem* indexer_ref) {
    printf("🔧 Initializing color sensor system...\n");
    
    // Store reference to indexer system
    indexer_system = indexer_ref;
    if (!indexer_system) {
        printf("❌ Indexer system reference is null\n");
        return false;
    }
    
    // Verify hardware objects exist
    if (!sensor1 || !sensor2) {
        printf("❌ Color sensor hardware objects not created properly\n");
        return false;
    }
    
    // Test sensor connectivity
    try {
        // Try to read from both sensors
        double prox1 = sensor1->get_proximity();
        double prox2 = sensor2->get_proximity();
        
        printf("📊 Sensor 1 proximity: %.2f\n", prox1);
        printf("📊 Sensor 2 proximity: %.2f\n", prox2);
        
        // Set LED brightness for better color detection
        sensor1->set_led_pwm(100); // 100% LED brightness
        sensor2->set_led_pwm(100); // 100% LED brightness
        
        printf("✅ Color sensor system initialized successfully\n");
        printf("🎯 Default mode: %s\n", sortingModeToString(current_mode));
        
        return true;
        
    } catch (const std::exception& e) {
        printf("❌ Error initializing color sensors: %s\n", e.what());
        return false;
    }
}

// =============================================================================
// MAIN UPDATE LOOP
// =============================================================================

void ColorSensorSystem::update() {
    if (!sensor1 || !sensor2 || !indexer_system) {
        return; // Hardware not available
    }
    
    uint32_t current_time = pros::millis();
    
    // Update sensor states
    bool sensor1_has_ball = isBallPresent(sensor1);
    bool sensor2_has_ball = isBallPresent(sensor2);
    
    // Handle sensor 1 state changes
    if (sensor1_has_ball && !sensor1_triggered) {
        // Ball just entered sensor 1
        sensor1_triggered = true;
        sensor1_trigger_time = current_time;
        printf("🔍 Ball detected at sensor 1\n");
    } else if (!sensor1_has_ball && sensor1_triggered) {
        // Ball just left sensor 1
        sensor1_triggered = false;
        printf("➡️ Ball left sensor 1\n");
    }
    
    // Handle sensor 2 state changes
    if (sensor2_has_ball && !sensor2_triggered) {
        // Ball just entered sensor 2
        sensor2_triggered = true;
        sensor2_trigger_time = current_time;
        printf("🔍 Ball detected at sensor 2\n");
    } else if (!sensor2_has_ball && sensor2_triggered) {
        // Ball just left sensor 2
        sensor2_triggered = false;
        printf("➡️ Ball left sensor 2\n");
    }
    
    // Perform color detection when balls are present
    if (sensor1_triggered) {
        BallColor color1 = readColorFromSensor(sensor1);
        BallColor confirmed_color1 = addToColorBuffer(1, color1);
        
        if (confirmed_color1 != BallColor::UNKNOWN && confirmed_color1 != BallColor::NO_BALL) {
            last_detected_color = confirmed_color1;
            
            // Update statistics
            if (confirmed_color1 == BallColor::RED) {
                red_balls_detected++;
            } else if (confirmed_color1 == BallColor::BLUE) {
                blue_balls_detected++;
            }
            
            printf("🎨 Ball color confirmed: %s\n", colorToString(confirmed_color1));
        }
    }
    
    if (sensor2_triggered) {
        BallColor color2 = readColorFromSensor(sensor2);
        BallColor confirmed_color2 = addToColorBuffer(2, color2);
        
        if (confirmed_color2 != BallColor::UNKNOWN && confirmed_color2 != BallColor::NO_BALL) {
            // Double-check color consistency
            if (last_detected_color != BallColor::UNKNOWN && 
                confirmed_color2 != last_detected_color) {
                printf("⚠️ Color mismatch between sensors: %s vs %s\n", 
                       colorToString(last_detected_color), colorToString(confirmed_color2));
                false_detections++;
            }
        }
    }
    
    // Determine ball direction
    last_direction = determineBallDirection();
    
    // Check if ball should be ejected
    if (last_detected_color != BallColor::UNKNOWN && 
        last_detected_color != BallColor::NO_BALL &&
        shouldEjectBall(last_detected_color)) {
        
        // Wait for ball to reach the ejection point (sensor 2 area)
        if (sensor2_triggered || 
            (current_time - sensor2_trigger_time < BALL_EJECT_DELAY_MS)) {
            startEjection();
        }
    }
    
    // Handle ejection timing
    if (ejection_active) {
        if (current_time - ejection_start_time >= ejection_duration) {
            stopEjection();
        }
    }
    
    // Reset detection state if ball passage times out
    if (sensor1_triggered && 
        (current_time - sensor1_trigger_time > BALL_PASSAGE_TIMEOUT_MS)) {
        printf("⏰ Sensor 1 detection timeout - resetting\n");
        sensor1_triggered = false;
        last_detected_color = BallColor::UNKNOWN;
    }
    
    if (sensor2_triggered && 
        (current_time - sensor2_trigger_time > BALL_PASSAGE_TIMEOUT_MS)) {
        printf("⏰ Sensor 2 detection timeout - resetting\n");
        sensor2_triggered = false;
    }
}

// =============================================================================
// PUBLIC INTERFACE METHODS
// =============================================================================

void ColorSensorSystem::setSortingMode(SortingMode mode) {
    current_mode = mode;
    printf("🎯 Sorting mode changed to: %s\n", sortingModeToString(mode));
}

SortingMode ColorSensorSystem::getSortingMode() const {
    return current_mode;
}

BallColor ColorSensorSystem::getLastDetectedColor() const {
    return last_detected_color;
}

BallDirection ColorSensorSystem::getLastDirection() const {
    return last_direction;
}

bool ColorSensorSystem::isBallDetected() const {
    return sensor1_triggered || sensor2_triggered;
}

void ColorSensorSystem::triggerEjection() {
    printf("🚀 Manual ejection triggered\n");
    startEjection();
}

void ColorSensorSystem::getStatistics(int& red_count, int& blue_count, 
                                      int& ejected_count, int& false_count) const {
    red_count = red_balls_detected;
    blue_count = blue_balls_detected;
    ejected_count = balls_ejected;
    false_count = false_detections;
}

void ColorSensorSystem::resetStatistics() {
    red_balls_detected = 0;
    blue_balls_detected = 0;
    balls_ejected = 0;
    false_detections = 0;
    printf("📊 Statistics reset\n");
}

void ColorSensorSystem::setEjectionDuration(uint32_t duration_ms) {
    // Clamp to safe limits
    if (duration_ms < BALL_EJECT_MIN_DURATION) {
        duration_ms = BALL_EJECT_MIN_DURATION;
        printf("⚠️ Ejection duration clamped to minimum: %dms\n", duration_ms);
    } else if (duration_ms > BALL_EJECT_MAX_DURATION) {
        duration_ms = BALL_EJECT_MAX_DURATION;
        printf("⚠️ Ejection duration clamped to maximum: %dms\n", duration_ms);
    }
    
    ejection_duration = duration_ms;
    printf("⏱️ Ejection duration set to: %dms\n", duration_ms);
}

uint32_t ColorSensorSystem::getEjectionDuration() const {
    return ejection_duration;
}

void ColorSensorSystem::printStatus() const {
    printf("\n=== COLOR SENSOR STATUS ===\n");
    printf("Mode: %s\n", sortingModeToString(current_mode));
    printf("Last Color: %s\n", colorToString(last_detected_color));
    printf("Last Direction: %s\n", directionToString(last_direction));
    printf("Sensor 1: %s\n", sensor1_triggered ? "TRIGGERED" : "CLEAR");
    printf("Sensor 2: %s\n", sensor2_triggered ? "TRIGGERED" : "CLEAR");
    printf("Ejection: %s\n", ejection_active ? "ACTIVE" : "INACTIVE");
    printf("Ejection Duration: %dms\n", ejection_duration);
    printf("Red Balls: %d\n", red_balls_detected);
    printf("Blue Balls: %d\n", blue_balls_detected);
    printf("Ejected: %d\n", balls_ejected);
    printf("False Detections: %d\n", false_detections);
    printf("===========================\n\n");
}

bool ColorSensorSystem::testSensors() {
    if (!sensor1 || !sensor2) {
        printf("❌ Sensors not initialized\n");
        return false;
    }
    
    try {
        // Test sensor 1
        double prox1 = sensor1->get_proximity();
        double hue1 = sensor1->get_hue();
        double sat1 = sensor1->get_saturation();
        double bright1 = sensor1->get_brightness();
        
        // Test sensor 2
        double prox2 = sensor2->get_proximity();
        double hue2 = sensor2->get_hue();
        double sat2 = sensor2->get_saturation();
        double bright2 = sensor2->get_brightness();
        
        printf("🔬 SENSOR TEST RESULTS:\n");
        printf("Sensor 1 - Prox: %.1f, Hue: %.1f°, Sat: %.1f, Bright: %.1f\n",
               prox1, hue1, sat1, bright1);
        printf("Sensor 2 - Prox: %.1f, Hue: %.1f°, Sat: %.1f, Bright: %.1f\n",
               prox2, hue2, sat2, bright2);
        
        return true;
        
    } catch (const std::exception& e) {
        printf("❌ Sensor test failed: %s\n", e.what());
        return false;
    }
}

// =============================================================================
// PRIVATE HELPER METHODS
// =============================================================================

BallColor ColorSensorSystem::readColorFromSensor(pros::Optical* sensor) {
    if (!sensor) return BallColor::UNKNOWN;
    
    try {
        double proximity = sensor->get_proximity();
        
        // Check if ball is present
        if (proximity > MAX_PROXIMITY_THRESHOLD) {
            return BallColor::NO_BALL;
        }
        
        double hue = sensor->get_hue();
        double saturation = sensor->get_saturation();
        double brightness = sensor->get_brightness();
        
        // Check minimum thresholds for valid color detection
        if (saturation < MIN_SATURATION || brightness < MIN_BRIGHTNESS) {
            return BallColor::UNKNOWN;
        }
        
        // Determine color based on hue
        if ((hue >= RED_HUE_MIN && hue <= RED_HUE_MAX) || 
            (hue >= RED_HUE_HIGH_MIN && hue <= RED_HUE_HIGH_MAX)) {
            return BallColor::RED;
        } else if (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX) {
            return BallColor::BLUE;
        }
        
        return BallColor::UNKNOWN;
        
    } catch (const std::exception& e) {
        printf("❌ Error reading sensor: %s\n", e.what());
        return BallColor::UNKNOWN;
    }
}

bool ColorSensorSystem::isBallPresent(pros::Optical* sensor) {
    if (!sensor) return false;
    
    try {
        double proximity = sensor->get_proximity();
        return proximity <= MAX_PROXIMITY_THRESHOLD;
    } catch (const std::exception& e) {
        return false;
    }
}

BallColor ColorSensorSystem::addToColorBuffer(int sensor_num, BallColor color) {
    BallColor* buffer = (sensor_num == 1) ? sensor1_color_buffer : sensor2_color_buffer;
    int& buffer_index = (sensor_num == 1) ? sensor1_buffer_index : sensor2_buffer_index;
    
    // Add color to buffer
    buffer[buffer_index] = color;
    buffer_index = (buffer_index + 1) % COLOR_CONFIRMATION_COUNT;
    
    // Check if all entries in buffer are the same (and not unknown/no_ball)
    if (color != BallColor::UNKNOWN && color != BallColor::NO_BALL) {
        bool all_same = true;
        for (int i = 0; i < COLOR_CONFIRMATION_COUNT; i++) {
            if (buffer[i] != color) {
                all_same = false;
                break;
            }
        }
        
        if (all_same) {
            return color; // Confirmed color
        }
    }
    
    return BallColor::UNKNOWN; // Not confirmed yet
}

BallDirection ColorSensorSystem::determineBallDirection() {
    uint32_t current_time = pros::millis();
    
    // Check timing between sensor triggers
    if (sensor1_trigger_time > 0 && sensor2_trigger_time > 0) {
        uint32_t time_diff = abs((int32_t)(sensor2_trigger_time - sensor1_trigger_time));
        
        if (time_diff < BALL_DIRECTION_TIMEOUT_MS) {
            if (sensor1_trigger_time < sensor2_trigger_time) {
                return BallDirection::FORWARD; // Sensor 1 -> Sensor 2
            } else {
                return BallDirection::REVERSE; // Sensor 2 -> Sensor 1
            }
        }
    }
    
    // Check if ball is stationary at one sensor
    if ((sensor1_triggered && !sensor2_triggered) || 
        (!sensor1_triggered && sensor2_triggered)) {
        return BallDirection::STATIONARY;
    }
    
    return BallDirection::UNKNOWN;
}

bool ColorSensorSystem::shouldEjectBall(BallColor color) {
    switch (current_mode) {
        case SortingMode::COLLECT_RED:
            return (color == BallColor::BLUE); // Eject blue, keep red
            
        case SortingMode::COLLECT_BLUE:
            return (color == BallColor::RED);  // Eject red, keep blue
            
        case SortingMode::COLLECT_ALL:
            return false; // Keep all balls
            
        case SortingMode::EJECT_ALL:
            return true;  // Eject all balls
            
        default:
            return false;
    }
}

void ColorSensorSystem::startEjection() {
    if (!ejection_active && indexer_system) {
        // Check if indexer system is currently being used for other operations
        // This prevents interference with driver-controlled scoring
        if (indexer_system->isScoringActive()) {
            printf("⚠️ Ejection delayed - indexer system is busy with scoring operation\n");
            return; // Don't eject if indexer is already active
        }
        
        printf("🚨 BALL EJECTION STARTING\n");
        printf("⏱️ Ejection duration: %dms\n", ejection_duration);
        
        // SAVE CURRENT STATE before ejection
        saveIndexerState();
        
        // Stop all current indexer operations before starting ejection
        indexer_system->stopAll();
        pros::delay(100); // Brief pause to ensure stop completes
        
        ejection_active = true;
        ejection_start_time = pros::millis();
        balls_ejected++;
        
        printf("🚀 Ball ejection started using indexer system (Total ejected: %d)\n", balls_ejected);
        
        // Temporarily set indexer to mid goal mode and execute back scoring
        indexer_system->setMidGoalMode();  // Set to mid goal scoring mode
        indexer_system->executeBack();     // Execute back indexer (ejects ball from back mid)
    }
}

void ColorSensorSystem::stopEjection() {
    if (ejection_active && indexer_system) {
        ejection_active = false;
        
        printf("⏹️ Ball ejection stopped - returning indexer to normal operation\n");
        
        // Stop all indexer motors to end the ejection sequence
        indexer_system->stopAll();
        
        // CRITICAL: Reset color sensor detection state after ejection
        // This prevents the system from thinking the ejected ball is still present
        resetDetectionState();
        
        // RESTORE PREVIOUS STATE if there was one
        restoreIndexerState();
        
        printf("🔄 Color sensor state reset and previous operation restored\n");
    }
}

// =============================================================================
// PRIVATE HELPER METHODS  
// =============================================================================

void ColorSensorSystem::resetDetectionState() {
    // Clear sensor trigger states
    sensor1_triggered = false;
    sensor2_triggered = false;
    sensor1_trigger_time = 0;
    sensor2_trigger_time = 0;
    
    // Reset color detection
    last_detected_color = BallColor::UNKNOWN;
    last_direction = BallDirection::UNKNOWN;
    
    // Clear color confirmation buffers
    for (int i = 0; i < COLOR_CONFIRMATION_COUNT; i++) {
        sensor1_color_buffer[i] = BallColor::NO_BALL;
        sensor2_color_buffer[i] = BallColor::NO_BALL;
    }
    sensor1_buffer_index = 0;
    sensor2_buffer_index = 0;
    
    printf("🧹 Detection state completely reset\n");
}

// =============================================================================
// STATE MANAGEMENT METHODS
// =============================================================================

void ColorSensorSystem::saveIndexerState() {
    if (!indexer_system) {
        saved_indexer_state.valid = false;
        return;
    }
    
    // Save current indexer system state
    saved_indexer_state.was_scoring_active = indexer_system->isScoringActive();
    saved_indexer_state.was_input_active = indexer_system->isInputActive();
    saved_indexer_state.saved_scoring_mode = static_cast<int>(indexer_system->getCurrentMode());
    saved_indexer_state.saved_execution_direction = static_cast<int>(indexer_system->getLastDirection());
    saved_indexer_state.valid = true;
    
    printf("💾 Indexer state saved: scoring=%s, input=%s, mode=%d, direction=%d\n",
           saved_indexer_state.was_scoring_active ? "ON" : "OFF",
           saved_indexer_state.was_input_active ? "ON" : "OFF",
           saved_indexer_state.saved_scoring_mode,
           saved_indexer_state.saved_execution_direction);
}

void ColorSensorSystem::restoreIndexerState() {
    if (!indexer_system || !saved_indexer_state.valid) {
        printf("⚠️ No valid state to restore\n");
        return;
    }
    
    printf("🔄 Restoring indexer state...\n");
    
    // Only restore if the indexer was actually doing something before
    if (saved_indexer_state.was_scoring_active) {
        // Small delay to ensure stopAll() has completed
        pros::delay(50);
        
        // Restore the scoring mode
        switch (saved_indexer_state.saved_scoring_mode) {
            case 0: // ScoringMode::COLLECTION
                indexer_system->setCollectionMode();
                break;
            case 1: // ScoringMode::MID_GOAL  
                indexer_system->setMidGoalMode();
                break;
            case 2: // ScoringMode::LOW_GOAL
                indexer_system->setLowGoalMode();
                break;
            case 3: // ScoringMode::TOP_GOAL
                indexer_system->setTopGoalMode();
                break;
            default:
                printf("⚠️ Unknown scoring mode: %d\n", saved_indexer_state.saved_scoring_mode);
                saved_indexer_state.valid = false;
                return;
        }
        
        // Restore the execution direction (restart the operation)
        switch (saved_indexer_state.saved_execution_direction) {
            case 1: // ExecutionDirection::FRONT
                printf("🔄 Resuming FRONT execution\n");
                indexer_system->executeFront();
                break;
            case 2: // ExecutionDirection::BACK
                printf("🔄 Resuming BACK execution\n");
                indexer_system->executeBack();
                break;
            case 3: // ExecutionDirection::STORAGE
                printf("🔄 Resuming STORAGE operation\n");
                indexer_system->startIntakeAndStorage();
                break;
            case 0: // ExecutionDirection::NONE
            default:
                // If direction was NONE, just ensure input is running if it was before
                if (saved_indexer_state.was_input_active) {
                    printf("🔄 Resuming input motor only\n");
                    indexer_system->startIntakeAndStorage(); // This starts intake
                }
                break;
        }
        
        printf("✅ Indexer state restored successfully\n");
    } else {
        printf("ℹ️ No active operation to restore\n");
    }
    
    // Mark state as used
    saved_indexer_state.valid = false;
}

// =============================================================================
// STRING CONVERSION UTILITIES
// =============================================================================

const char* ColorSensorSystem::colorToString(BallColor color) const {
    switch (color) {
        case BallColor::RED: return "RED";
        case BallColor::BLUE: return "BLUE";
        case BallColor::NO_BALL: return "NO_BALL";
        case BallColor::UNKNOWN:
        default: return "UNKNOWN";
    }
}

const char* ColorSensorSystem::directionToString(BallDirection direction) const {
    switch (direction) {
        case BallDirection::FORWARD: return "FORWARD";
        case BallDirection::REVERSE: return "REVERSE";
        case BallDirection::STATIONARY: return "STATIONARY";
        case BallDirection::UNKNOWN:
        default: return "UNKNOWN";
    }
}

const char* ColorSensorSystem::sortingModeToString(SortingMode mode) const {
    switch (mode) {
        case SortingMode::COLLECT_RED: return "COLLECT_RED";
        case SortingMode::COLLECT_BLUE: return "COLLECT_BLUE";
        case SortingMode::COLLECT_ALL: return "COLLECT_ALL";
        case SortingMode::EJECT_ALL: return "EJECT_ALL";
        default: return "UNKNOWN_MODE";
    }
}