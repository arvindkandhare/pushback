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
    
    // Initialize LemLib first (safe to call multiple times)
    printf("🔧 Calling LemLib initialization...\n");
    initializeLemLib();
    
    // Validate that all LemLib objects were created successfully
    if (!validateLemLibInitialization()) {
        printf("❌ FATAL ERROR: LemLib initialization failed!\n");
        printf("❌ Cannot continue - robot will not function properly\n");
        return;
    }
    
    printf("✅ LemLib verified and ready\n");
    
    // Create controller
    master = new pros::Controller(pros::E_CONTROLLER_MASTER);
    
    // Create PTO system
    pto_system = new PTO();
    
    // Create drivetrain (now uses LemLib motor references) - ONLY after LemLib is validated
    custom_drivetrain = new Drivetrain(pto_system);
    
    // Create subsystems that depend on other systems
    indexer_system = new IndexerSystem(pto_system);
    intake_system = new Intake();
    autonomous_system = new AutonomousSystem(pto_system, indexer_system);
    
    // Engage PTO to lift middle wheels (reduces friction during testing)
    printf("Lifting middle wheels via PTO...\n");
    pto_system->setScorerMode();  // This lifts/disconnects middle wheels
    
    printf("Global subsystems initialized!\n");
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
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
	
	// Brief delay to check competition status
	pros::delay(100);
	
	// Autonomous selection for development mode ONLY (when no competition switch)
	if (!pros::competition::is_connected()) {
		printf("Development Mode: 10 seconds for autonomous selection\n");
		
		// Show available modes on controller
		master->set_text(0, 0, "AUTO SELECT");
		master->set_text(1, 0, "UP/DN: change A: ok");
		
		// Allow 10 seconds for selection in development mode
		int countdown = 500; // 500 * 20ms = 10 seconds
		while (countdown > 0) {
			autonomous_system->getSelector().update();
			
			// Update countdown display every 0.5 seconds
			if (countdown % 25 == 0) {
				master->print(1, 0, "A:ok %ds left", countdown / 50);
			}
			
			countdown--;
			pros::delay(20);
		}
		
		master->set_text(0, 0, "READY");
		master->set_text(1, 0, "L1+L2 to test");
		pros::delay(1000);
		
		printf("Development selection complete. Use L1+L2 to test or R1+R2 to change.\n");
	} else {
		printf("Competition mode detected - selection will happen in disabled() period\n");
		master->set_text(0, 0, "COMPETITION");
		master->set_text(1, 0, "Select in disabled");
		pros::delay(1000);
	}
	
	printf("=== INITIALIZATION COMPLETE ===\n");
}

/**
 * Display ASCII art status on controller
 */
void displayControllerArt(const char* mode, const char* status) {
	if (!master || !master->is_connected()) return;
	
	// Different ASCII art based on mode
	if (strcmp(mode, "AWP") == 0) {
		master->set_text(0, 0, "AWP MODE   [*]   ");
		master->set_text(1, 0, status);
	} else if (strcmp(mode, "SKILLS") == 0) {
		master->set_text(0, 0, "SKILLS    \\o/   ");
		master->set_text(1, 0, status);
	} else if (strcmp(mode, "TEST") == 0) {
		master->set_text(0, 0, "TEST      <->    ");
		master->set_text(1, 0, status);
	} else if (strcmp(mode, "SUCCESS") == 0) {
		master->set_text(0, 0, "SUCCESS    :)    ");
		master->set_text(1, 0, "   [✓] Done      ");
	} else if (strcmp(mode, "ERROR") == 0) {
		master->set_text(0, 0, "ERROR      :(    ");
		master->set_text(1, 0, "   [X] Failed    ");
	} else if (strcmp(mode, "LOADING") == 0) {
		master->set_text(0, 0, "WORKING...       ");
		master->set_text(1, 0, " [=====>    ]    ");
	} else {
		// Default
		master->set_text(0, 0, mode);
		master->set_text(1, 0, status);
	}
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
		while (countdown > 0 && !autonomous_system->getSelector().isModeConfirmed()) {
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
		// pros::delay(1000);  // REMOVED: No delay after selection
	} else {
		// Competition mode - continuous loop during disabled period
		while (pros::competition::is_disabled() && !autonomous_system->getSelector().isModeConfirmed()) {
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
	
	// CRITICAL: Ensure PTO is in scorer mode (pistons UP) for autonomous
	printf("🔧 Pre-flight check: Setting PTO to scorer mode...\n");
	if (pto_system) {
		pto_system->setScorerMode();  // Pistons UP - disconnects middle wheels
		pros::delay(300);  // Allow pneumatics time to fully actuate
		printf("✅ PTO pistons UP - middle wheels disconnected for scoring\n");
	}
	
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
			
			printf("=== L1+L2 PRESSED - TESTING AUTONOMOUS ===\n");
			
			// Check if autonomous system is properly initialized
			if (!autonomous_system) {
				printf("ERROR: autonomous_system is NULL!\n");
				master->set_text(0, 0, "ERROR: NO AUTO SYS");
				pros::delay(2000);
				continue;
			}
			
			// Get currently selected mode
			AutoMode current_mode = autonomous_system->getSelector().getSelectedMode();
			printf("Current selected mode: %d\n", static_cast<int>(current_mode));
			
			// Show current selection and confirm
			master->set_text(0, 0, "TEST CURRENT AUTO");
			master->print(1, 0, "Mode: %d", static_cast<int>(current_mode));
			
			// Wait for buttons to be released
			while (master->get_digital(pros::E_CONTROLLER_DIGITAL_L1) || 
				   master->get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
				pros::delay(20);
			}
			
			// Show test execution warning with ASCII art
			displayControllerArt("LOADING", "Stand Clear!");
			pros::delay(2000);
			
			// PREPARE FOR AUTONOMOUS TRANSITION
			printf("=== PREPARING AUTONOMOUS TRANSITION ===\n");
			
			// 1. Stop all current drivetrain activity
			custom_drivetrain->stop();
			printf("✓ Stopped custom drivetrain\n");
			
			// 2. Ensure PTO is in correct state for autonomous (SCORING MODE - middle wheels for scoring)
			if (pto_system) {
				if (pto_system->isDrivetrainMode()) {
					printf("Setting PTO to scorer mode for autonomous (middle wheels for scoring)...\n");
					pto_system->setScorerMode(); // This puts middle wheels in scoring position
					pros::delay(300); // Allow pneumatics to actuate
				}
				printf("✓ PTO in scorer mode (autonomous uses 2-wheel drive + middle wheels for scoring)\n");
			}
			
			// 3. Re-initialize LemLib if needed (fresh start)
			printf("Re-initializing LemLib for clean autonomous start...\n");
			chassis->setPose(0, 0, 0); // Reset odometry
			pros::delay(100);
			printf("✓ LemLib reset complete\n");
			
			printf("About to call autonomous_system->runAutonomous()\n");
			// Run autonomous
			autonomous_system->runAutonomous();
			printf("Returned from autonomous_system->runAutonomous()\n");
			
			// RESTORE OPCONTROL STATE
			printf("=== RESTORING OPCONTROL STATE ===\n");
			
			// 1. Stop any residual LemLib activity
			// LemLib should handle this automatically, but ensure motors are stopped
			pros::delay(100);
			
			// 2. Reset custom drivetrain brake modes for opcontrol
			custom_drivetrain->setBrakeMode(DRIVETRAIN_BRAKE_MODE); // Coast mode
			printf("✓ Restored opcontrol brake mode\n");
			
			// Back to driver control with success indicator
			displayControllerArt("SUCCESS", "Test Complete!");
			pros::delay(2000);
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