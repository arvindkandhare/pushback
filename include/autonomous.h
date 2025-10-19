/**
 * \file autonomous.h
 *
 * Autonomous system header file for VEX Push Back 2024-25.
 * Implements odometry-based navigation with AWP-focused routines.
 */

#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

#include "api.h"
#include "config.h"
#include "drivetrain.h"
#include "pto.h"
#include "indexer.h"
#include <cmath>

/**
 * Position structure for robot coordinates
 */
struct Position {
    double x;      // X coordinate in inches
    double y;      // Y coordinate in inches  
    double heading; // Heading in degrees (0 = facing positive X)
    
    Position(double x = 0, double y = 0, double heading = 0) 
        : x(x), y(y), heading(heading) {}
};

/**
 * PID Controller class for movement control
 */
class PIDController {
private:
    double kp, ki, kd;
    double previous_error;
    double integral;
    uint32_t last_time;
    
public:
    PIDController(double kp, double ki, double kd);
    double calculate(double error);
    void reset();
};

/**
 * Autonomous Selector class for LCD-based mode selection
 */
class AutoSelector {
private:
    AutoMode selected_mode;
    int selector_position;
    bool mode_confirmed;
    
public:
    AutoSelector();
    void displayOptions();
    void handleInput();
    AutoMode getSelectedMode();
    bool isModeConfirmed();
    void update();
};

/**
 * Main Autonomous System class
 * Handles odometry tracking, navigation, and autonomous routines
 */
class AutonomousSystem {
private:
    // Hardware references
    pros::adi::Encoder vertical_encoder;
    pros::adi::Encoder horizontal_encoder;
    pros::Imu gyro;
    
    Drivetrain* drivetrain;
    PTO* pto_system;
    IndexerSystem* indexer_system;
    AutoSelector auto_selector;
    
    // Position tracking
    Position current_position;
    Position target_position;
    
    // Movement control
    PIDController drive_pid;
    PIDController turn_pid;
    
    // State tracking
    bool odometry_initialized;
    bool autonomous_running;
    uint32_t last_update_time;
    
    // Internal methods
    void updateOdometry();
    double normalizeAngle(double angle);
    double getDistanceToTarget();
    double getAngleToTarget();
    double getHeadingError(double target_heading);
    
public:
    /**
     * Constructor - initializes all sensors and systems
     */
    AutonomousSystem(Drivetrain* dt, PTO* pto, IndexerSystem* indexer);
    
    /**
     * Initialize autonomous system - call during initialize()
     */
    void initialize();
    
    /**
     * Update odometry and selector - call continuously in disabled/competition_initialize
     */
    void update();
    
    /**
     * Set robot position (for calibration)
     */
    void setPosition(double x, double y, double heading);
    
    /**
     * Get current robot position
     */
    Position getPosition();
    
    /**
     * Navigation functions
     */
    void driveToPoint(double x, double y, double max_speed = DRIVE_MAX_SPEED, double timeout_ms = 5000);
    void turnToHeading(double heading, double max_speed = TURN_MAX_SPEED, double timeout_ms = 3000);
    void driveDistance(double distance, double heading = -999, double max_speed = DRIVE_MAX_SPEED, double timeout_ms = 5000);
    
    /**
     * Autonomous route functions
     */
    void executeRedLeftAWP();
    void executeRedLeftBonus();
    void executeRedRightAWP();
    void executeRedRightBonus();
    void executeSkillsRoutine();
    
    /**
     * Main autonomous entry point - call from autonomous()
     */
    void runAutonomous();
    
    /**
     * Utility functions
     */
    bool isMovementComplete();
    void stopAllMovement();
    void printPosition();
    
    /**
     * Get the autonomous selector for external access
     */
    AutoSelector& getSelector();
    
    /**
     * Testing and calibration functions
     */
    void testStraightDrive(double distance = 24.0);
    void testTurnAccuracy(double angle = 90.0);
    void testPointToPoint();
    void testOdometryAccuracy();
    void printSensorReadings();
};

#endif // _AUTONOMOUS_H_