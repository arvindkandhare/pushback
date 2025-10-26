/**
 * \file visual_status.cpp
 *
 * Enhanced visual status display system implementation.
 * Provides graphical and symbolic representations of robot state.
 */

#include "visual_status.h"
#include <cstring>  // for strcmp, strcpy, memset

// Visual symbols for instant recognition
const char* VisualStatusDisplay::DRIVE_SYMBOLS[] = {
    "🚗3WHL ████",    // 3-wheel drive (drivetrain mode)
    "🎯2WHL ██▒▒"     // 2-wheel drive (scorer mode)
};

const char* VisualStatusDisplay::MODE_SYMBOLS[] = {
    "❌ NONE  ----",     // No mode selected - NEEDS SETUP
    "🔄 COLLECT READY",   // Collection mode - READY
    "🎯 MID-GL READY",   // Mid goal mode - READY  
    "⬇ LOW-GL READY",    // Low goal mode - READY
    "⬆ TOP-GL READY"     // Top goal mode - READY
};

const char* VisualStatusDisplay::STATUS_SYMBOLS[] = {
    "⏸ READY",          // Ready state
    "▶ ACTIVE",         // Active state
    "⚡ FRONT",          // Front direction
    "⚡ BACK"            // Back direction
};

VisualStatusDisplay::VisualStatusDisplay() 
    : last_update(0), force_update(true) {
    
    // Initialize display lines
    for (int i = 0; i < 3; i++) {
        memset(display_lines[i], 0, sizeof(display_lines[i]));
    }
}

void VisualStatusDisplay::updateDisplay(pros::Controller& controller, 
                                      const PTO* pto, 
                                      const IndexerSystem* indexer) {
    
    if (!controller.is_connected()) return;
    
    uint32_t current_time = pros::millis();
    
    // Check if update is needed
    if (!force_update && (current_time - last_update < REFRESH_MS)) {
        return;
    }
    
    char new_lines[3][17];
    
    // LINE 0: Drive Mode Status with Visual Bar
    snprintf(new_lines[0], 17, "%s", 
             pto->isDrivetrainMode() ? DRIVE_SYMBOLS[0] : DRIVE_SYMBOLS[1]);
    
    // LINE 1: Scoring Mode with Symbol
    int mode_index = 0;
    switch(indexer->getCurrentMode()) {
        case ScoringMode::NONE:       mode_index = 0; break;
        case ScoringMode::COLLECTION: mode_index = 1; break;
        case ScoringMode::MID_GOAL:   mode_index = 2; break;
        case ScoringMode::LOW_GOAL:   mode_index = 3; break;
        case ScoringMode::TOP_GOAL:   mode_index = 4; break;
    }
    snprintf(new_lines[1], 17, "%s", MODE_SYMBOLS[mode_index]);
    
    // LINE 2: Setup Status and Configuration Readiness
    if (indexer->getCurrentMode() == ScoringMode::NONE) {
        snprintf(new_lines[2], 17, "! SETUP NEEDED");
    } else if (indexer->isScoringActive()) {
        const char* direction = (indexer->getLastDirection() == ExecutionDirection::FRONT) ? "F" : "B";
        snprintf(new_lines[2], 17, "> SCORING %s", direction);
    } else {
        // Show what's configured and ready
        const char* storage = (pto->isScorerMode()) ? "SCOR" : "DRIVE";
        snprintf(new_lines[2], 17, "* %s READY", storage);
    }
    
    // Update only changed lines to reduce flicker
    for (int i = 0; i < 3; i++) {
        if (strcmp(new_lines[i], display_lines[i]) != 0 || force_update) {
            controller.print(i, 0, "%s", new_lines[i]);
            strcpy(display_lines[i], new_lines[i]);
        }
    }
    
    last_update = current_time;
    force_update = false;
}

void VisualStatusDisplay::forceUpdate() {
    force_update = true;
}

void VisualStatusDisplay::clearDisplay(pros::Controller& controller) {
    if (!controller.is_connected()) return;
    
    for (int i = 0; i < 3; i++) {
        controller.clear_line(i);
        memset(display_lines[i], 0, sizeof(display_lines[i]));
    }
}

void VisualStatusDisplay::provideModeChangeFeedback(pros::Controller& controller, ScoringMode mode) {
    if (!controller.is_connected()) return;
    
    switch(mode) {
        case ScoringMode::COLLECTION:
            controller.rumble(".");      // Single short: Collection
            break;
        case ScoringMode::MID_GOAL:
            controller.rumble("..");     // Double short: Mid Goal
            break;
        case ScoringMode::LOW_GOAL:
            controller.rumble("...");    // Triple short: Low Goal
            break;
        case ScoringMode::TOP_GOAL:
            controller.rumble("-");      // Single long: Top Goal
            break;
        case ScoringMode::NONE:
            controller.rumble("- -");    // Long-pause-long: Disabled
            break;
    }
}

void VisualStatusDisplay::providePTOChangeFeedback(pros::Controller& controller, bool drive_mode) {
    if (!controller.is_connected()) return;
    
    if (drive_mode) {
        controller.rumble(".-.");   // Morse 'R': Ready to drive
    } else {
        controller.rumble("-.-");   // Morse 'K': Ready to score (oK)
    }
}

void VisualStatusDisplay::provideActivationFeedback(pros::Controller& controller) {
    if (!controller.is_connected()) return;
    
    controller.rumble("--");        // Long rumble for activation
}