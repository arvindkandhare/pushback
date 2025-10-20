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
	
	// Initialize autonomous system (includes gyro calibration)
	autonomous_system->initialize();
	
	printf("=== INITIALIZATION COMPLETE ===\n");
}


	

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	// Update autonomous selector while disabled
	static int update_counter = 0;
	
	// Update selector every 100ms
	if (update_counter % 5 == 0) {
		autonomous_system->update();
	}
	update_counter++;
	
	pros::delay(20);
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
	printf("Use LCD to select autonomous mode\n");
	
	// The autonomous selector will display on LCD
	// Update continuously until autonomous starts
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
	printf("=== AUTONOMOUS STARTED ===\n");
	
	// Display selected mode
	AutoMode mode = autonomous_system->getSelector().getSelectedMode();
	printf("Selected autonomous mode: %d\n", static_cast<int>(mode));
	
	// Run the selected autonomous routine
	autonomous_system->runAutonomous();
	
	printf("=== AUTONOMOUS COMPLETE ===\n");
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
	
	// Global subsystems are already created at file scope
	
	// Initialize LCD for opcontrol mode
	pros::lcd::set_text(0, "OPCONTROL ACTIVE");
	pros::lcd::set_text(1, "Tank Drive Ready");
	
	static int counter = 0;
	static int lcd_update_counter = 0;
	
	// Main control loop
	bool auton_ran = false;
	while (true) {
		counter++;
		lcd_update_counter++;

		// TEMP: Hold L1+L2 for 1500ms to run autonomous ONCE for testing (reduces accidental presses)
		static uint32_t hold_start = 0;
		bool l1 = master->get_digital(pros::E_CONTROLLER_DIGITAL_L1);
		bool l2 = master->get_digital(pros::E_CONTROLLER_DIGITAL_L2);
		if (l1 && l2) {
			if (hold_start == 0) hold_start = pros::millis();
			else if (!auton_ran && pros::millis() - hold_start >= 1500) {
				printf("[TEST] L1+L2 held: Running autonomous routine!\n");
				pros::lcd::set_text(2, "[TEST] Running Auton");
				autonomous();
				auton_ran = true;
			}
		} else {
			hold_start = 0;
		}

		// Print debug info every second (50Hz * 50 = 1 second)
		if (counter % 50 == 0) {
			printf("OPCONTROL LOOP: %d seconds\n", counter / 50);
		}

		// Update LCD less frequently to avoid conflicts (every 2 seconds)
		if (lcd_update_counter >= 100) {
			lcd_update_counter = 0;

			// Check controller connection and update LCD
			if (master->is_connected()) {
				master->print(0, 0, "Connected: %d", counter / 50);
			} else {
				pros::lcd::set_text(1, "Controller DISCONNECTED");
				printf("Controller DISCONNECTED!\n");
			}
		}

		// Update all subsystems - this is where button mappings are handled
		custom_drivetrain->update(*master);
		pto_system->update(*master);
		indexer_system->update(*master);
		intake_system->update(*master);  // Update intake system

		pros::delay(20);  // 50Hz loop
	}
}