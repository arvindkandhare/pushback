/**
 * \file drivetrain.cpp
 *
 * Drivetrain control system implementation.
 * Manages 6-wheel tank drive with PTO support for switching between 2 and 3 wheel drive.
 */

#include "drivetrain.h"

Drivetrain::Drivetrain(PTO* pto) 
    : left_front(*left_front_motor),     // Reference to existing LemLib motor
      left_middle(*left_middle_motor),   // Reference to existing LemLib motor
      left_back(*left_back_motor),       // Reference to existing LemLib motor
      right_front(*right_front_motor),   // Reference to existing LemLib motor
      right_middle(*right_middle_motor), // Reference to existing LemLib motor
      right_back(*right_back_motor),     // Reference to existing LemLib motor
      pto_system(pto) {
    
    // Set brake mode for all motors
    setBrakeMode(DRIVETRAIN_BRAKE_MODE);
}

void Drivetrain::tankDrive(int left_power, int right_power) {
    // Apply deadzone to inputs
    left_power = applyDeadzone(left_power, JOYSTICK_DEADZONE);
    right_power = applyDeadzone(right_power, JOYSTICK_DEADZONE);
    
    // Apply sensitivity scaling
    left_power = (int)(left_power * TANK_DRIVE_SENSITIVITY);
    right_power = (int)(right_power * TANK_DRIVE_SENSITIVITY);
    
    // Always drive front and back wheels
    left_front.move(left_power);
    left_back.move(left_power);
    right_front.move(right_power);
    right_back.move(right_power);
    
    // Drive middle wheels only if PTO is in drivetrain mode
    if (pto_system && pto_system->isDrivetrainMode()) {
        left_middle.move(left_power);
        right_middle.move(right_power);
    }
    // Note: When in scorer mode, don't control middle wheels at all
    // Let the IndexerSystem/scorer mechanism control them
}

void Drivetrain::update(pros::Controller& controller) {
    // Get tank drive inputs from controller
    int left_stick = controller.get_analog(TANK_DRIVE_LEFT_STICK);
    int right_stick = controller.get_analog(TANK_DRIVE_RIGHT_STICK);
    
    // Apply tank drive
    tankDrive(left_stick, right_stick);
    
    // Display drive mode on LCD
    // LCD call removed to prevent rendering conflicts
    
    // Display motor powers for debugging
    // LCD call removed to prevent rendering conflicts
}

void Drivetrain::stop() {
    // Stop all motors
    left_front.move(0);
    left_middle.move(0);
    left_back.move(0);
    right_front.move(0);
    right_middle.move(0);
    right_back.move(0);
}

void Drivetrain::setBrakeMode(pros::v5::MotorBrake brake_mode) {
    // Set brake mode for all motors
    left_front.set_brake_mode(brake_mode);
    left_middle.set_brake_mode(brake_mode);
    left_back.set_brake_mode(brake_mode);
    right_front.set_brake_mode(brake_mode);
    right_middle.set_brake_mode(brake_mode);
    right_back.set_brake_mode(brake_mode);
}

int Drivetrain::applyDeadzone(int input, int deadzone) {
    // Apply deadzone - if input is within deadzone range, return 0
    if (abs(input) < deadzone) {
        return 0;
    }
    return input;
}

const char* Drivetrain::getCurrentDriveModeString() const {
    if (pto_system && pto_system->isDrivetrainMode()) {
        return "3-Wheel Drive";
    } else {
        return "2-Wheel Drive";
    }
}
