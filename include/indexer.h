/**
 * \file indexer.h
 *
 * Indexer and scoring system header file.
 * Manages ball intake, indexing, and scoring for both front and back directions.
 */

#ifndef _INDEXER_H_
#define _INDEXER_H_

#include "api.h"
#include "config.h"
#include "pto.h"

/**
 * Scoring mode enumeration - defines what happens when execution button is pressed
 */
enum class ScoringMode {
    COLLECTION,     ///< Collection/intake mode - run for ball collection only
    MID_GOAL,       ///< Mid level scoring 
    LOW_GOAL,       ///< Low goal scoring - only intake motor in reverse
    TOP_GOAL,       ///< Top level scoring
    NONE            ///< No mode selected
};

/**
 * Execution direction enumeration - which button executes the selected mode
 */
enum class ExecutionDirection {
    FRONT,      ///< Execute with front indexer (R2)
    BACK,       ///< Execute with back indexer (R1)
    STORAGE,    ///< NEW: Intake and storage operation
    NONE        ///< No execution yet
};

/**
 * IndexerSystem class
 * 
 * This class manages the complete ball handling system including:
 * - Input motor for ball intake
 * - Front indexer motor
 * - Back indexer (via PTO system)
 * - Scoring direction and level selection
 * - Automated scoring sequences
 */
class IndexerSystem {
private:
    // Motors
    pros::Motor input_motor;        ///< 11W motor for ball intake at bottom
    pros::Motor top_indexer;        ///< Top indexer motor (shared between front/back top scoring)
    
    // Pneumatic systems
    pros::adi::Pneumatics front_flap;  ///< Pneumatic control for front scoring flap
    
    // PTO system reference for back indexer control
    PTO* pto_system;
    
    // Current scoring configuration
    ScoringMode current_mode;            ///< Currently selected scoring mode
    ExecutionDirection last_direction;   ///< Last execution direction used
    
    // State tracking
    bool scoring_active;            ///< True when scoring sequence is running
    uint32_t scoring_start_time;    ///< Time when scoring sequence started
    uint32_t input_start_time;      ///< Time when input motor started
    bool input_motor_active;        ///< True when input motor is running
    bool score_from_top_storage;    ///< True when scoring from top storage is enabled
    bool front_flap_open;           ///< True when front flap is open (manual tracking)
    
    // Button state tracking (for edge detection)
    bool last_collection_button;
    bool last_mid_goal_button;
    bool last_low_goal_button;
    bool last_top_goal_button;
    bool last_front_execute_button;
    bool last_back_execute_button;
    bool last_storage_toggle_button;
    bool last_front_flap_toggle_button;  ///< For direct front flap control

    // Display management
    char last_displayed_line0[17];      ///< Last content displayed on line 0
    char last_displayed_line1[17];      ///< Last content displayed on line 1  
    char last_displayed_line2[17];      ///< Last content displayed on line 2
    uint32_t last_display_update;       ///< Time of last display update
    bool force_display_update;          ///< Force display update on next cycle

public:
    /**
     * Constructor
     * @param pto Pointer to the PTO system for back indexer control
     */
    IndexerSystem(PTO* pto);

    /**
     * Set scoring mode to collection/intake
     */
    void setCollectionMode();

    /**
     * Set scoring mode to mid goal scoring
     */
    void setMidGoalMode();

    /**
     * Set scoring mode to low goal scoring (intake reverse only)
     */
    void setLowGoalMode();

    /**
     * Set scoring mode to top goal scoring
     */
    void setTopGoalMode();

    /**
     * Execute selected mode with front indexer (R2 button)
     */
    void executeFront();

    /**
     * Execute selected mode with back indexer (R1 button)
     */
    void executeBack();

    /**
     * Open front flap to allow balls to score
     */
    void openFrontFlap();

    /**
     * Close front flap to hold balls against it
     */
    void closeFrontFlap();

    /**
     * Toggle front flap between open and closed (direct control)
     */
    void toggleFrontFlap();

    /**
     * Start input motor for ball intake
     */
    void startInput();

    /**
     * Start input motor in reverse for low goal scoring
     */
    void startInputReverse();

    /**
     * NEW: Start intake and move balls to storage
     * Called when any mode button (X, A, B, Y) is pressed
     */
    void startIntakeAndStorage();

    /**
     * Stop input motor
     */
    void stopInput();

    /**
     * Stop all motors and reset scoring state
     */
    void stopAll();

    /**
     * Get current scoring mode
     * @return Current scoring mode
     */
    ScoringMode getCurrentMode() const;

    /**
     * Get last execution direction
     * @return Last execution direction used
     */
    ExecutionDirection getLastDirection() const;

    /**
     * Check if scoring sequence is currently active
     * @return True if scoring sequence is running
     */
    bool isScoringActive() const;

    /**
     * Check if input motor is currently active
     * @return True if input motor is running
     */
    bool isInputActive() const;

    /**
     * Update indexer system - call this in opcontrol loop
     * Handles button press detection and automatic timeouts
     * @param controller Reference to the master controller
     */
    void update(pros::Controller& controller);

    /**
     * Check if a flow can be interrupted
     * @return True if current flow can be safely interrupted
     */
    bool canInterruptFlow() const;

    /**
     * Get flow status information
     * @return String describing current flow status
     */
    const char* getFlowStatus() const;

    /**
     * Update controller display with current status
     * @param controller Reference to the master controller
     * @param force_update Force immediate update regardless of timing
     */
    void updateControllerDisplay(pros::Controller& controller, bool force_update = false);

    /**
     * Get string representation of current mode for debugging
     * @return "Collection", "Mid Goal", "Low Goal", "Top Goal", or "None"
     */
    const char* getModeString() const;

    /**
     * Get string representation of last direction for debugging
     * @return "Front", "Back", or "None"
     */
    const char* getDirectionString() const;

  

    /**
     * Stop left indexer motor - for testing only
     */
    void stopLeftIndexer();

    /**
     * Stop right indexer motor - for testing only
     */
    void stopRightIndexer();

    /**
     * Toggle score from top storage mode on/off
     */
    void toggleStorageMode();

    /**
     * Get current storage mode state
     * @return True if scoring from top storage is enabled
     */
    bool isStorageModeActive() const;

    /**
     * Verify PTO is in correct mode for storage operations
     * @return True if PTO is ready for storage operations
     */
    bool verifyPTOForStorage();

private:
    /**
     * Run left indexer (left middle motor via PTO) for front operations
     * @param speed Motor speed in RPM (positive or negative)
     */
    void runLeftIndexer(int speed);

    /**
     * Run right indexer (right middle motor via PTO) for back operations
     * @param speed Motor speed in RPM (positive or negative)
     */
    void runRightIndexer(int speed);

    /**
     * Run top indexer motor (shared for front/back top scoring)
     * @param speed Motor speed in RPM (positive or negative)
     */
    void runTopIndexer(int speed);

    /**
     * Stop top indexer motor
     */
    void stopTopIndexer();

    /**
     * Format mode for compact display
     * @return Single character representing mode
     */
    char getModeChar() const;

    /**
     * Format direction for compact display  
     * @return Single character representing direction
     */
    char getDirectionChar() const;

    /**
     * Format status icon for display
     * @return Status symbol character
     */
    char getStatusIcon() const;
};

#endif // _INDEXER_H_