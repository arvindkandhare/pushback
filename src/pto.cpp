/**
 * \file pto.cpp
 *
 * PTO (Power Take-Off) system implementation.
 * Manages pneumatic cylinders that switch between drivetrain and scorer modes.
 */

#include "pto.h"

PTO::PTO() 
    : left_pneumatic(PTO_LEFT_PNEUMATIC),
      right_pneumatic(PTO_RIGHT_PNEUMATIC),
      current_state(PTO_DEFAULT_STATE),
      last_button_state(false) {
    
    // Set initial PTO state
    if (current_state == PTO_EXTENDED) {
        setDrivetrainMode();
    } else {
        setScorerMode();
    }
}

void PTO::setDrivetrainMode() {
    // Extend pneumatics - connect middle wheels to drivetrain
    left_pneumatic.set_value(PTO_EXTENDED);
    right_pneumatic.set_value(PTO_EXTENDED);
    current_state = PTO_EXTENDED;
    
    // Allow pneumatics time to actuate (critical for proper operation)
    pros::delay(250);
    
    // Debug output
    // LCD call removed to prevent rendering conflicts
}

void PTO::setScorerMode() {
    // Retract pneumatics - connect middle wheels to scorer
    left_pneumatic.set_value(PTO_RETRACTED);
    right_pneumatic.set_value(PTO_RETRACTED);
    current_state = PTO_RETRACTED;
    
    // Allow pneumatics time to actuate (critical for proper operation)
    pros::delay(250);
    
    // Debug output
    // LCD call removed to prevent rendering conflicts
}

void PTO::toggle() {
    if (current_state == PTO_EXTENDED) {
        setScorerMode();
    } else {
        setDrivetrainMode();
    }
}

bool PTO::isDrivetrainMode() const {
    return current_state == PTO_EXTENDED;
}

bool PTO::isScorerMode() const {
    return current_state == PTO_RETRACTED;
}

void PTO::update(pros::Controller& controller) {
    // Get current button state
    bool current_button_state = controller.get_digital(PTO_TOGGLE_BUTTON);
    
    // Check for button press (rising edge detection)
    if (current_button_state && !last_button_state) {
        toggle();
        
        // Provide haptic feedback
        controller.rumble(".");
    }
    
    // Update last button state for next iteration
    last_button_state = current_button_state;
}

const char* PTO::getCurrentModeString() const {
    return (current_state == PTO_EXTENDED) ? "Drivetrain" : "Scorer";
}
