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
 * Scoring direction enumeration
 */
enum class ScoringDirection {
    FRONT,  ///< Score towards front of robot
    BACK,   ///< Score towards back of robot (using PTO)
    NONE    ///< No direction selected
};

/**
 * Scoring level enumeration
 */
enum class ScoringLevel {
    LONG_GOAL,  ///< Top scoring (higher goal)
    MID_GOAL,   ///< Bottom scoring (lower goal)
    NONE        ///< No level selected
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
    pros::Motor front_indexer;      ///< Dedicated motor for front indexer
    
    // PTO system reference for back indexer control
    PTO* pto_system;
    
    // Current scoring configuration
    ScoringDirection current_direction;  ///< Currently selected scoring direction
    ScoringLevel current_level;          ///< Currently selected scoring level
    
    // State tracking
    bool scoring_active;            ///< True when scoring sequence is running
    uint32_t scoring_start_time;    ///< Time when scoring sequence started
    uint32_t input_start_time;      ///< Time when input motor started
    bool input_motor_active;        ///< True when input motor is running
    
    // Button state tracking (for edge detection)
    bool last_front_button;
    bool last_back_button;
    bool last_long_goal_button;
    bool last_mid_goal_button;
    bool last_execute_button;
    bool last_input_button;

public:
    /**
     * Constructor
     * @param pto Pointer to the PTO system for back indexer control
     */
    IndexerSystem(PTO* pto);

    /**
     * Set scoring direction to front
     */
    void setFrontScoring();

    /**
     * Set scoring direction to back (requires PTO in scorer mode)
     */
    void setBackScoring();

    /**
     * Set scoring level to long goal (top scoring)
     */
    void setLongGoal();

    /**
     * Set scoring level to mid goal (bottom scoring)
     */
    void setMidGoal();

    /**
     * Execute scoring sequence based on current direction and level
     * Will automatically configure PTO if needed for back scoring
     */
    void executeScoring();

    /**
     * Start input motor for ball intake
     */
    void startInput();

    /**
     * Stop input motor
     */
    void stopInput();

    /**
     * Stop all motors and reset scoring state
     */
    void stopAll();

    /**
     * Get current scoring direction
     * @return Current scoring direction
     */
    ScoringDirection getCurrentDirection() const;

    /**
     * Get current scoring level  
     * @return Current scoring level
     */
    ScoringLevel getCurrentLevel() const;

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
     * Get string representation of current direction for debugging
     * @return "Front", "Back", or "None"
     */
    const char* getDirectionString() const;

    /**
     * Get string representation of current level for debugging
     * @return "Long Goal", "Mid Goal", or "None"
     */
    const char* getLevelString() const;

    // Testing functions for individual indexer control
    /**
     * Test left indexer motor (left middle wheel) - for testing only
     * @param speed Motor speed in RPM (positive or negative)
     */
    void testLeftIndexer(int speed);

    /**
     * Test right indexer motor (right middle wheel) - for testing only  
     * @param speed Motor speed in RPM (positive or negative)
     */
    void testRightIndexer(int speed);

    /**
     * Stop left indexer motor - for testing only
     */
    void stopLeftIndexer();

    /**
     * Stop right indexer motor - for testing only
     */
    void stopRightIndexer();

private:
    /**
     * Run front indexer at specified speed and direction
     * @param speed Motor speed in RPM (positive or negative)
     */
    void runFrontIndexer(int speed);

    /**
     * Run back indexer at specified speed and direction
     * Requires PTO to be in scorer mode
     * @param speed Motor speed in RPM (positive or negative)
     */
    void runBackIndexer(int speed);

    /**
     * Stop front indexer
     */
    void stopFrontIndexer();

    /**
     * Stop back indexer
     */
    void stopBackIndexer();

    /**
     * Check if scoring sequence has timed out
     * @return True if scoring should stop due to timeout
     */
    bool checkScoringTimeout();

    /**
     * Check if input motor has timed out
     * @return True if input motor should stop due to timeout
     */
    bool checkInputTimeout();
};

#endif // _INDEXER_H_