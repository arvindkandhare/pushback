/**
 * \file intake.cpp
 *
 * Intake mechanism system implementation.
 * Manages pneumatic cylinder that controls intake extension/retraction for ball collection.
 */

#include "intake.h"

Intake::Intake() 
    : intake_pneumatic(INTAKE_PNEUMATIC),
      current_state(INTAKE_DEFAULT_STATE),
      last_button_state(false) {
    
    // Set initial intake state
    if (current_state == INTAKE_EXTENDED) {
        extend();
    } else {
        retract();
    }
}

void Intake::retract() {
    // Retract pneumatic - store intake mechanism
    intake_pneumatic.set_value(INTAKE_RETRACTED);
    current_state = INTAKE_RETRACTED;
    
    // Debug output
    printf("Intake: RETRACTED (stored position)\n");
}

void Intake::extend() {
    // Extend pneumatic - deploy intake mechanism for ball collection
    intake_pneumatic.set_value(INTAKE_EXTENDED);
    current_state = INTAKE_EXTENDED;
    
    // Debug output
    printf("Intake: EXTENDED (deployed for collection)\n");
}

void Intake::toggle() {
    if (current_state == INTAKE_EXTENDED) {
        retract();
    } else {
        extend();
    }
}

bool Intake::isExtended() const {
    return current_state == INTAKE_EXTENDED;
}

bool Intake::isRetracted() const {
    return current_state == INTAKE_RETRACTED;
}

void Intake::update(pros::Controller& controller) {
    // Get current button state
    bool current_button_state = controller.get_digital(INTAKE_TOGGLE_BUTTON);
    
    // Check for button press (rising edge detection)
    if (current_button_state && !last_button_state) {
        toggle();
        
        // Provide haptic feedback - different pattern from PTO
        controller.rumble("..");
    }
    
    // Update last button state for next iteration
    last_button_state = current_button_state;
}

const char* Intake::getCurrentStateString() const {
    return (current_state == INTAKE_EXTENDED) ? "Extended" : "Retracted";
}