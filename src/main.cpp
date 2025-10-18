/**
 * \file main.cpp
 *
 * Main control file for the pushback robot.
 * 
 * This file contains the main robot control functions including:
 * - Robot initialization
 * - Autonomous routine (placeholder for future development)
 * - Driver control (opcontrol) with tank drive and PTO system
 * 
 * Robot Configuration:
 * - 6-wheel tank drive (3 wheels per side)
 * - 3.75" omni wheels
 * - 11W motors with green cartridges (18:1 gearing)
 * - PTO system for switching middle wheels between drive and scorer
 * - Pneumatic cylinders control PTO engagement/disengagement
 */

#include "main.h"
#include "indexer.h"


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// Initialize LCD once and set basic startup message
	pros::lcd::initialize();
	pros::lcd::set_text(0, "Pushback Robot");
	pros::lcd::set_text(1, "Initializing...");
	
	// Brief delay for initialization
	printf("Robot initializing...\n");
	pros::delay(1000);
	
	printf("=== INITIALIZATION COMPLETE ===\n");
}


	

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	// Simple console message, avoid LCD conflicts
	printf("Robot DISABLED - waiting for enable...\n");
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	// Competition-specific initialization can go here
	// For example: autonomous routine selector, alliance color selection, etc.
	
	printf("Competition Mode - Pushback Robot Ready\n");
	// Only update LCD if not in use by other functions
	pros::lcd::set_text(0, "Competition Mode");
	pros::lcd::set_text(1, "Robot Ready");
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// TODO: Implement autonomous routines
	// This is a placeholder for future autonomous development
	
	printf("Autonomous Mode - No routine implemented\n");
	pros::lcd::set_text(0, "Autonomous Mode");
	pros::lcd::set_text(1, "No routine");
	
	// Example autonomous actions (commented out):
	// 1. Drive forward for 2 seconds
	// drivetrain.tankDrive(100, 100);
	// pros::delay(2000);
	// drivetrain.stop();
	
	// 2. Toggle PTO and use scorer
	// pto_system.setScorerMode();
	// pros::delay(1000);
	// pto_system.setDrivetrainMode();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the operator control task
 * will be stopped. Re-enabling the robot will restart the task, not resume it
 * from where it left off.
 */
void opcontrol() {
	printf("=== OPCONTROL STARTED ===\n");
	
	// Global robot subsystems
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	PTO pto_system;
	Drivetrain drivetrain(&pto_system);
	IndexerSystem indexer_system(&pto_system);
	Intake intake_system;  // New intake mechanism
	
	// Initialize LCD for opcontrol mode
	pros::lcd::set_text(0, "OPCONTROL ACTIVE");
	pros::lcd::set_text(1, "Tank Drive Ready");
	
	static int counter = 0;
	static int lcd_update_counter = 0;
	
	// Main control loop
	while (true) {
		counter++;
		lcd_update_counter++;
		
		// Print debug info every second (50Hz * 50 = 1 second)
		if (counter % 50 == 0) {
			printf("OPCONTROL LOOP: %d seconds\n", counter / 50);
		}
		
		// Update LCD less frequently to avoid conflicts (every 2 seconds)
		if (lcd_update_counter >= 100) {
			lcd_update_counter = 0;
			
			// Check controller connection and update LCD
			if (master.is_connected()) {
				master.print(0, 0, "Connected: %d", counter / 50);
			} else {
				pros::lcd::set_text(1, "Controller DISCONNECTED");
				printf("Controller DISCONNECTED!\n");
			}
		}
		
		// Update all subsystems - this is where button mappings are handled
		drivetrain.update(master);
		pto_system.update(master);
		indexer_system.update(master);
		intake_system.update(master);  // Update intake system
		
		pros::delay(20);  // 50Hz loop
	}
}