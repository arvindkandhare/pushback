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
#include "intake.h"
#include "autonomous.h"
#include "lemlib_config.h"

// Global robot subsystems (pointers to avoid early construction)
pros::Controller* master = nullptr;
PTO* pto_system = nullptr;
Drivetrain* custom_drivetrain = nullptr;
IndexerSystem* indexer_system = nullptr;
Intake* intake_system = nullptr;
AutonomousSystem* autonomous_system = nullptr;

/**
 * Initialize all global subsystems.
 * This creates objects after the VEX system is properly initialized.
 */
void initializeGlobalSubsystems() {
    printf("Initializing global subsystems...\n");
    
    // Initialize LemLib first (this ensures motor objects exist)
    initializeLemLib();
    
    // Create controller
    master = new pros::Controller(pros::E_CONTROLLER_MASTER);
    
    // Create PTO system
    pto_system = new PTO();
    
    // Create drivetrain (now uses LemLib motor references)
    custom_drivetrain = new Drivetrain(pto_system);
    
    // Create subsystems that depend on other systems
    indexer_system = new IndexerSystem(pto_system);
    intake_system = new Intake();
    autonomous_system = new AutonomousSystem(custom_drivetrain, pto_system, indexer_system);
    
    printf("Global subsystems initialized!\n");
}

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
	pros::delay(500);
	
	// Initialize global subsystems FIRST (after VEX system is ready)
	initializeGlobalSubsystems();
	
	// Controller display for initialization status
	master->set_text(0, 0, "GYRO CAL...");
	
	// Initialize autonomous system (includes gyro calibration)
	autonomous_system->initialize();
	
	// Display completion on controller
	master->set_text(0, 0, "INIT DONE");
	
	printf("=== INITIALIZATION COMPLETE ===\n");
}


	

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	printf("=== DISABLED MODE - AUTONOMOUS SELECTION ===\n");
	
	// Test competition API
	printf("Competition API status: %s\n", 
		   pros::competition::is_connected() ? "Connected" : "Not Connected");
	
	// If no competition switch, allow selection for a limited time (development mode)
	if (!pros::competition::is_connected()) {
		printf("Development Mode: 10 seconds for autonomous selection\n");
		master->set_text(0, 0, "DEV MODE");
		master->set_text(1, 0, "10s to select");
		
		// Allow 10 seconds for selection in development mode
		int countdown = 500; // 500 * 20ms = 10 seconds
		while (countdown > 0) {
			autonomous_system->getSelector().update();
			
			// Update countdown display
			if (countdown % 25 == 0) { // Update every 0.5 seconds
				master->print(1, 0, "%ds to select", countdown / 50);
			}
			
			countdown--;
			pros::delay(20);
		}
		
		master->set_text(0, 0, "SELECTION DONE");
		master->set_text(1, 0, "Starting...");
		pros::delay(1000);
	} else {
		// Competition mode - continuous loop during disabled period
		while (pros::competition::is_disabled()) {
			// Update autonomous selector (UP/DOWN/A buttons work here)
			autonomous_system->getSelector().update();
			
			// Small delay to prevent overwhelming the system
			pros::delay(20);
		}
	}
	
	printf("=== EXITING DISABLED - STARTING OPERATION ===\n");
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
	printf("=== COMPETITION INITIALIZE ===\n");
	printf("Connected to Competition Switch/FMS\n");
	printf("Pushback Robot Ready for Competition\n");
	
	// Display instructions on controller
	master->set_text(0, 0, "COMPETITION MODE");
	master->set_text(1, 0, "Select Auto Mode");
	
	// Competition-specific setup
	printf("Use controller UP/DOWN/A to select autonomous mode\n");
	printf("Selection available during DISABLED period\n");
	
	printf("=== COMPETITION INITIALIZE COMPLETE ===\n");
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	printf("=== AUTONOMOUS PERIOD STARTED ===\n");
	
	// Display autonomous start on controller
	master->set_text(0, 0, "AUTON RUNNING");
	
	// Get and display selected mode
	AutoMode mode = autonomous_system->getSelector().getSelectedMode();
	printf("Executing autonomous mode: %d\n", static_cast<int>(mode));
	
	// Show mode on controller
	master->print(1, 0, "Mode: %d", static_cast<int>(mode));
	
	// Run the selected autonomous routine
	autonomous_system->runAutonomous();
	
	// Display completion on controller
	master->set_text(0, 0, "AUTON COMPLETE");
	
	printf("=== AUTONOMOUS PERIOD COMPLETE ===\n");
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
	printf("=== DRIVER CONTROL PERIOD STARTED ===\n");
	
	// Display opcontrol start on controller
	master->set_text(0, 0, "DRIVER CONTROL");
	master->set_text(1, 0, "Good Luck!");
	master->rumble("-.-"); // Short-long-short rumble
	
	// Timer for periodic updates
	static int counter = 0;
	static int lcd_update_counter = 0;
	
	// Main driver control loop
	while (true) {
		counter++;
		lcd_update_counter++;

		// Check for autonomous testing (L1+L2 = run selected autonomous)
		if (master->get_digital(pros::E_CONTROLLER_DIGITAL_L1) && 
			master->get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			
			// Get currently selected mode
			AutoMode current_mode = autonomous_system->getSelector().getSelectedMode();
			
			// Show current selection and confirm
			master->set_text(0, 0, "TEST CURRENT AUTO");
			master->print(1, 0, "Mode: %d", static_cast<int>(current_mode));
			
			// Wait for buttons to be released
			while (master->get_digital(pros::E_CONTROLLER_DIGITAL_L1) || 
				   master->get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
				pros::delay(20);
			}
			
			// Show test execution warning
			master->set_text(0, 0, "RUNNING AUTO!");
			master->set_text(1, 0, "Stand Clear!");
			pros::delay(2000);
			
			// Run autonomous
			autonomous_system->runAutonomous();
			
			// Back to driver control
			master->set_text(0, 0, "DRIVER CONTROL");
			master->set_text(1, 0, "Auto Test Done");
			pros::delay(1000);
		}

		// Check for autonomous mode change (R1+R2 = change autonomous mode)
		if (master->get_digital(pros::E_CONTROLLER_DIGITAL_R1) && 
			master->get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			
			// Allow autonomous mode selection during driver control
			master->set_text(0, 0, "CHANGE AUTO MODE");
			master->set_text(1, 0, "Use UP/DOWN/A");
			
			while (master->get_digital(pros::E_CONTROLLER_DIGITAL_R1) || 
				   master->get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
				autonomous_system->getSelector().update();
				pros::delay(20);
			}
			
			// Show completion
			master->set_text(0, 0, "MODE CHANGED");
			master->set_text(1, 0, "L1+L2 to test");
			pros::delay(2000);
		}

		// Print debug info every 10 seconds (50Hz * 500 = 10 seconds)
		if (counter % 500 == 0) {
			printf("DRIVER CONTROL: %d seconds elapsed\n", counter / 50);
		}

		// Update controller display every 2 seconds (50Hz * 100 = 2 seconds)
		if (lcd_update_counter >= 100) {
			lcd_update_counter = 0;

			// Check controller connection and update display
			if (master->is_connected()) {
				master->print(0, 0, "Time: %ds", counter / 50);
			} else {
				printf("WARNING: Controller DISCONNECTED!\n");
			}
		}

		// Update all robot subsystems - this handles button mappings
		custom_drivetrain->update(*master);
		pto_system->update(*master);
		indexer_system->update(*master);
		intake_system->update(*master);  // Update intake system
		
		// Small delay to prevent overwhelming the system
		pros::delay(20);  // 50Hz loop
	}
	
	printf("=== DRIVER CONTROL PERIOD ENDED ===\n");
}