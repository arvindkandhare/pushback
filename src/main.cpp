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
	
	// Try to get LCD working
	pros::lcd::initialize();
	pros::lcd::set_text(0, "INITIALIZE WORKS!");
	pros::lcd::set_text(1, "Line 2 test");
	
	// Wait and then start opcontrol immediately
	printf("Starting opcontrol in 1 second...\n");
	pros::delay(10000);
	
	printf("=== CALLING OPCONTROL ===\n");
}


	

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	
	
	// Display disabled status
	pros::lcd::print(0, "Robot DISABLED");
	pros::lcd::print(1, "Waiting for enable...");
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
	
	pros::lcd::print(0, "Competition Mode");
	pros::lcd::print(1, "Pushback Robot Ready");
	pros::lcd::print(2, "Waiting for match start...");
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
	
	pros::lcd::print(0, "Autonomous Mode");
	pros::lcd::print(1, "No autonomous routine");
	pros::lcd::print(2, "Implement later...");
	
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
	printf("=== OPCONTROL STARTED ===\n");
	printf("=== OPCONTROL STARTED ===\n");
// Global robot subsystems
pros::Controller master(pros::E_CONTROLLER_MASTER);
PTO pto_system;
Drivetrain drivetrain(&pto_system);
IndexerSystem indexer_system(&pto_system);
	
	pros::lcd::print(0, "OPCONTROL ACTIVE!");
	
	static int counter = 0;
	
	// Simplified main loop
	while (true) {
		counter++;
		
		// Print every second (50Hz * 50 = 1 second)
		if (counter % 50 == 0) {
			printf("OPCONTROL LOOP: %d seconds\n", counter / 50);
			pros::lcd::print(1, "Loop: %d", counter / 50);
		}
		
		// Test controller connection
		if (master.is_connected()) {
			if (counter % 100 == 0) {  // Every 2 seconds
				printf("Controller is connected\n");
				master.print(0, 0, "Connected: %d", counter / 50);
			}
		} else {
			printf("Controller DISCONNECTED!\n");
		}
		
		// Update all subsystems - this is where button mappings are handled
		drivetrain.update(master);
		pto_system.update(master);
		indexer_system.update(master);
		
		pros::delay(20);  // 50Hz loop
	}
}