/**
 * \file drivetrain.h
 *
 * Drivetrain control system header file.
 * Manages 6-wheel tank drive with PTO support for switching between 2 and 3 wheel drive.
 */

#ifndef _DRIVETRAIN_H_
#define _DRIVETRAIN_H_

#include "api.h"
#include "config.h"
#include "pto.h"

/**
 * Drivetrain class
 * 
 * This class manages the 6-wheel tank drive system with support for PTO (Power Take-Off).
 * The robot has 3 wheels per side, with the middle wheels switchable between drivetrain
 * and scorer mechanism via pneumatic PTO system.
 * 
 * Drive Modes:
 * - 3-wheel drive: All wheels connected to drivetrain (PTO extended)
 * - 2-wheel drive: Only front/back wheels for drive, middle wheels for scorer (PTO retracted)
 */
class Drivetrain {
private:
    // Left side motors
    pros::Motor left_front;   ///< Left front drive motor
    pros::Motor left_middle;  ///< Left middle drive motor (PTO switchable)
    pros::Motor left_back;    ///< Left back drive motor

    // Right side motors  
    pros::Motor right_front;  ///< Right front drive motor
    pros::Motor right_middle; ///< Right middle drive motor (PTO switchable)
    pros::Motor right_back;   ///< Right back drive motor

    PTO* pto_system;          ///< Pointer to PTO system for mode checking

public:
    /**
     * Constructor - initializes all drivetrain motors
     * @param pto Pointer to the PTO system for mode checking
     */
    Drivetrain(PTO* pto);

    /**
     * Tank drive control function
     * @param left_power Left side power (-127 to 127)
     * @param right_power Right side power (-127 to 127)
     */
    void tankDrive(int left_power, int right_power);

    /**
     * Update drivetrain - call this in opcontrol loop
     * Handles tank drive control based on controller input
     * @param controller Reference to the master controller
     */
    void update(pros::Controller& controller);

    /**
     * Stop all drivetrain motors
     */
    void stop();

    /**
     * Set brake mode for all drivetrain motors
     * @param brake_mode The brake mode to set
     */
    void setBrakeMode(pros::v5::MotorBrake brake_mode);

    /**
     * Apply deadzone to joystick input
     * @param input Raw joystick input
     * @param deadzone Deadzone threshold
     * @return Processed input with deadzone applied
     */
    static int applyDeadzone(int input, int deadzone);

    /**
     * Get the current drive mode based on PTO state
     * @return "3-Wheel Drive" or "2-Wheel Drive" depending on PTO state
     */
    const char* getCurrentDriveModeString() const;
};

#endif // _DRIVETRAIN_H_