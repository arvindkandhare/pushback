/**
 * Simple integration example for the enhanced controller visual display.
 * 
 * This shows how to integrate the visual status system into your main.cpp
 * to provide instant setup verification for the driver.
 */

// 1. Add this include to main.cpp:
#include "visual_status.h"

// 2. Add this global variable with your other globals:
VisualStatusDisplay* visual_display = nullptr;

// 3. In initializeGlobalSubsystems(), add:
visual_display = new VisualStatusDisplay();

// 4. In opcontrol() function, replace display logic with:
void opcontrol() {
    printf("=== DRIVER CONTROL PERIOD STARTED ===\n");
    
    // Clear controller and show startup
    visual_display->clearDisplay(*master);
    master->set_text(0, 0, "DRIVER CONTROL");
    master->set_text(1, 0, "Good Luck!");
    master->rumble("-.-");
    
    // Give driver a moment to see startup
    pros::delay(1000);
    
    // Main driver control loop
    while (true) {
        // Update all subsystems (your existing code)
        custom_drivetrain->update(*master);
        pto_system->update(*master);
        indexer_system->update(*master);
        intake_system->update(*master);
        
        // UPDATE: Replace old controller display with visual system
        visual_display->updateDisplay(*master, pto_system, indexer_system);
        
        // ADD: Haptic feedback for mode changes
        static ScoringMode last_mode = ScoringMode::NONE;
        ScoringMode current_mode = indexer_system->getCurrentMode();
        if (current_mode != last_mode) {
            VisualStatusDisplay::provideModeChangeFeedback(*master, current_mode);
            last_mode = current_mode;
        }
        
        // ADD: Haptic feedback for PTO changes  
        static bool last_pto_state = true;
        bool current_pto_state = pto_system->isDrivetrainMode();
        if (current_pto_state != last_pto_state) {
            VisualStatusDisplay::providePTOChangeFeedback(*master, current_pto_state);
            last_pto_state = current_pto_state;
        }
        
        // Rest of your existing opcontrol logic here...
        
        pros::delay(20);
    }
}