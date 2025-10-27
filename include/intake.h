/**
 * \file intake.h
 *
 * Front match loader system header file.
 * Manages motor and rotational encoder that controls front match loader deployment for ball collection.
 */

#ifndef _INTAKE_H_
#define _INTAKE_H_

#include "api.h"
#include "config.h"

/**
 * Intake class
 * 
 * This class manages the motor and rotational encoder that controls the front match loader mechanism.
 * The front match loader helps collect balls from the match load tubes.
 * 
 * Hardware:
 * - Motor: Port 7 (V5 Smart Motor)
 * - Position Sensor: ADI port E (Analog position sensor - potentiometer/encoder)
 * - Gear Ratio: 12:1 (6-tooth driving gear to 72-tooth driven gear)
 * 
 * States:
 * - RETRACTED (default): Front match loader is stored/retracted (vertical, 0°)
 * - DEPLOYED: Front match loader is deployed for ball collection (-110°)
 */
class Intake {
private:
    pros::Motor front_loader_motor;              ///< Front match loader motor (port 7)
    pros::adi::AnalogIn front_loader_sensor;     ///< Front loader position sensor (potentiometer or similar)
    bool front_loader_deployed;                 ///< Current state (true = deployed, false = retracted)
    double front_loader_target_position;        ///< Target position in loader degrees (not motor degrees)
    bool last_button_state;                     ///< Last state of toggle button (for edge detection)
    double sensor_zero_value;                   ///< Calibrated zero position sensor reading

public:
    /**
     * Constructor - initializes front loader motor and encoder
     */
    Intake();

    /**
     * Set front loader to retracted position (default/stored position)
     * Front loader is stored vertically and out of the way
     */
    void retract();

    /**
     * Set front loader to deployed position
     * Front loader is deployed for ball collection from match load tubes
     */
    void deploy();

    /**
     * Toggle between retracted and deployed positions
     */
    void toggle();

    /**
     * Get current front loader state
     * @return true if deployed, false if retracted
     */
    bool isDeployed() const;

    /**
     * Get current front loader state
     * @return true if retracted, false if deployed
     */
    bool isRetracted() const;

    /**
     * Get current front loader position from encoder
     * @return Current position in loader degrees (not motor degrees)
     */
    double getPosition() const;

    /**
     * Get current motor position from encoder  
     * @return Current motor position in motor degrees
     */
    double getMotorPosition() const;

    /**
     * Check if front loader is at target position
     * @return True if loader is within tolerance of target position
     */
    bool isAtTarget() const;

    /**
     * Update intake system - call this in opcontrol loop
     * Handles button press detection for toggling and position control
     * @param controller Reference to the master controller
     */
    void update(pros::Controller& controller);

    /**
     * Get string representation of current state for debugging
     * @return "Deployed" or "Retracted" depending on current state
     */
    const char* getCurrentStateString() const;

    /**
     * Print comprehensive debug information about the front loader
     */
    void printDebugInfo() const;

    /**
     * Reset encoder to zero at current position (for calibration)
     */
    void resetEncoder();

    /**
     * Recalibrate sensor zero position at current location
     */
    void calibrateSensorZero();

    /**
     * Manual calibration - call this when loader is in known position
     * @param current_position The actual position of the loader arm in degrees
     */
    void calibratePosition(double current_position);

private:
    /**
     * Set front loader to specific position
     * @param target_degrees Target position in loader degrees (not motor degrees)
     */
    void setPosition(double target_degrees);

    /**
     * Convert loader degrees to motor degrees using gear ratio
     * @param loader_degrees Position in loader degrees
     * @return Position in motor degrees
     */
    double loaderDegreesToMotorDegrees(double loader_degrees) const;

    /**
     * Convert motor degrees to loader degrees using gear ratio
     * @param motor_degrees Position in motor degrees  
     * @return Position in loader degrees
     */
    double motorDegreesToLoaderDegrees(double motor_degrees) const;
};

#endif // _INTAKE_H_