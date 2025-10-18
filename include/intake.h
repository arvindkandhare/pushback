/**
 * \file intake.h
 *
 * Intake mechanism system header file.
 * Manages pneumatic cylinder that controls intake extension/retraction for ball collection.
 */

#ifndef _INTAKE_H_
#define _INTAKE_H_

#include "api.h"
#include "config.h"

/**
 * Intake class
 * 
 * This class manages the pneumatic cylinder that controls the intake mechanism.
 * The intake mechanism helps collect balls from the intake tubes.
 * 
 * States:
 * - RETRACTED (default): Intake mechanism is stored/retracted
 * - EXTENDED: Intake mechanism is deployed for ball collection
 */
class Intake {
private:
    pros::adi::DigitalOut intake_pneumatic;  ///< Intake pneumatic cylinder
    bool current_state;                      ///< Current intake state (true = extended, false = retracted)
    bool last_button_state;                  ///< Last state of toggle button (for edge detection)

public:
    /**
     * Constructor - initializes intake pneumatic
     */
    Intake();

    /**
     * Set intake to retracted position (default/stored position)
     * Intake mechanism is stored and out of the way
     */
    void retract();

    /**
     * Set intake to extended position
     * Intake mechanism is deployed for ball collection from tubes
     */
    void extend();

    /**
     * Toggle between retracted and extended positions
     */
    void toggle();

    /**
     * Get current intake state
     * @return true if extended, false if retracted
     */
    bool isExtended() const;

    /**
     * Get current intake state
     * @return true if retracted, false if extended
     */
    bool isRetracted() const;

    /**
     * Update intake system - call this in opcontrol loop
     * Handles button press detection for toggling
     * @param controller Reference to the master controller
     */
    void update(pros::Controller& controller);

    /**
     * Get string representation of current state for debugging
     * @return "Extended" or "Retracted" depending on current state
     */
    const char* getCurrentStateString() const;
};

#endif // _INTAKE_H_