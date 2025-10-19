# VEX Push Back 2024-25 - Autonomous Strategy

## Table of Contents
1. [Game Analysis](#game-analysis)
2. [Robot Capabilities](#robot-capabilities)
3. [AWP Strategy](#awp-strategy)
4. [Autonomous Routes](#autonomous-routes)
5. [Implementation Plan](#implementation-plan)
6. [Scoring Analysis](#scoring-analysis)

---

## Game Analysis

### VEX Push Back 2024-25 Overview
- **Field**: 12' x 12' square
- **Match Time**: 15s Autonomous + 1:45 Driver Control
- **Objects**: 88 Blocks total
- **Goals**: 4 total (2 Long Goals + 2 Center Goals)
- **Park Zones**: 2 (one per alliance)

### Scoring System
| Action | Points | Notes |
|--------|---------|--------|
| Block Scored | 3 pts | Each block in any goal |
| Long Goal Zone Control | 10 pts | Per controlled zone |
| Center Goal Upper Control | 8 pts | Upper section control |
| Center Goal Lower Control | 6 pts | Lower section control |
| 1 Robot Parked | 8 pts | Single robot in park zone |
| 2 Robots Parked | 30 pts | Both alliance robots parked |
| **Autonomous Bonus** | **10 pts** | **Higher scoring alliance in auto** |
| **AWP (Both alliances)** | **1 Win Point** | **50% of match victory** |

### Strategic Priorities
1. **AWP Achievement** - Critical (50% match win)
2. **Autonomous Bonus** - Secondary (10 points)
3. **Reliable Execution** - Consistency over complexity

---

## Robot Capabilities

### Hardware Configuration
- **Drivetrain**: 6-wheel tank (3 per side) with PTO system
- **Wheels**: 3.75" omni wheels with 11W motors (18:1 green cartridge)
- **Scoring**: Front/back dual-direction scoring system
- **Intake**: Pneumatic extension/retraction mechanism
- **Storage**: 6-7 block capacity

### Sensor Suite
- **Odometry**: Vertical (port 9) + Horizontal (port 10) tracking wheels
- **Gyroscope**: Inertial sensor (port 13) for heading control
- **Positioning**: Accurate autonomous navigation capability

### Scoring Mechanisms
- **Front Scoring**: Left middle motor + pneumatic flap + top indexer
- **Back Scoring**: Right middle motor + top indexer (reversed)
- **Intake**: Bottom input motor + pneumatic extension
- **PTO System**: Switch between 3-wheel drive and 2-wheel scorer modes

---

## AWP Strategy

### AWP Requirements Analysis
*Note: Specific AWP tasks need verification from latest game manual v2.1 (released 10/9/2025)*

**Likely AWP Tasks (to be confirmed):**
1. Touch/reach park zone during autonomous
2. Score blocks in specific goals
3. Control certain field zones
4. Complete sequence within time limit

### Priority Framework
1. **Reliable AWP completion** - Must achieve consistently (90%+ success rate)
2. **Safe execution** - Avoid risky maneuvers that could fail AWP
3. **Bonus points secondary** - Only pursue if AWP is secure

---

## Autonomous Routes

### Red Alliance Positions

#### Red Left Starting Position
**Route RL1: AWP-Focused (Conservative)**
```
Time | Action | Details
-----|--------|--------
0-2s | Initialize | Set PTO to scorer mode, deploy intake
2-5s | Drive to Goal | Move to nearest scoring position
5-8s | Score Blocks | Deposit pre-loaded blocks (1-2 blocks)
8-12s | Complete AWP | Perform AWP task (exact task TBD)
12-15s | Position | Set up for driver control period
```

**Route RL2: AWP + Bonus (Aggressive)**
```
Time | Action | Details
-----|--------|--------
0-1s | Quick Setup | Fast PTO toggle, intake deploy
1-4s | Rush Score | Quick score 2-3 blocks in nearest goal
4-7s | Collect More | Grab additional blocks if accessible
7-10s | Second Score | Attempt additional scoring
10-13s | Complete AWP | Perform AWP task (exact task TBD)
13-15s | Position | Set up for driver control period
```

#### Red Right Starting Position
**Route RR1: AWP-Focused (Conservative)**
```
Time | Action | Details
-----|--------|--------
0-2s | Initialize | Set PTO to scorer mode, deploy intake
2-5s | Drive to Goal | Move to nearest scoring position
5-8s | Score Blocks | Deposit pre-loaded blocks (1-2 blocks)
8-12s | Complete AWP | Perform AWP task (exact task TBD)
12-15s | Position | Set up for driver control period
```

**Route RR2: Mirror RL2** *(Adjusted for right starting position)*

---

## Implementation Plan

### Phase 1: Core Movement System
```cpp
class AutonomousSystem {
private:
    // Odometry tracking
    pros::adi::Encoder vertical_encoder;   // Port 9
    pros::adi::Encoder horizontal_encoder; // Port 10
    pros::Imu gyro;                        // Port 13
    
    // Robot systems
    Drivetrain* drivetrain;
    PTO* pto_system;
    IndexerSystem* indexer;
    
    // Position tracking
    double robot_x, robot_y, robot_heading;
    
public:
    void updateOdometry();
    void driveToPoint(double target_x, double target_y);
    void turnToHeading(double target_heading);
    void executeAWPRoute();
    void executeBonusRoute();
};
```

### Phase 2: Autonomous Selector
```cpp
enum class AutoMode {
    RED_LEFT_AWP,      // Conservative AWP route
    RED_LEFT_BONUS,    // Aggressive AWP + bonus route
    RED_RIGHT_AWP,     // Conservative AWP route
    RED_RIGHT_BONUS,   // Aggressive AWP + bonus route
    SKILLS,            // Programming skills routine
    DISABLED           // No autonomous
};

class AutoSelector {
private:
    AutoMode selected_mode;
    int selector_position;
    
public:
    void displayOptions();
    void handleInput();
    AutoMode getSelectedMode();
};
```

### Phase 3: Route Execution
```cpp
void autonomous() {
    auto_selector.displayOptions();
    AutoMode mode = auto_selector.getSelectedMode();
    
    switch(mode) {
        case AutoMode::RED_LEFT_AWP:
            executeRedLeftAWP();
            break;
        case AutoMode::RED_LEFT_BONUS:
            executeRedLeftBonus();
            break;
        case AutoMode::RED_RIGHT_AWP:
            executeRedRightAWP();
            break;
        case AutoMode::RED_RIGHT_BONUS:
            executeRedRightBonus();
            break;
        case AutoMode::SKILLS:
            executeSkillsRoutine();
            break;
        default:
            // No autonomous
            break;
    }
}
```

---

## Scoring Analysis

### Expected Autonomous Points

#### Conservative AWP Route
- **Pre-loaded blocks**: 2 blocks × 3 pts = 6 pts
- **AWP achievement**: Win Point (50% match value)
- **Total**: 6 pts + AWP
- *Note: Parking only counts at end of full match, not during autonomous*

#### Aggressive AWP + Bonus Route
- **Scored blocks**: 4-5 blocks × 3 pts = 12-15 pts
- **Autonomous bonus**: 10 pts (if we outscore opponent)
- **AWP achievement**: Win Point (50% match value)
- **Total**: 22-25 pts + AWP
- *Note: Parking only counts at end of full match, not during autonomous*

### Risk Assessment
| Route | AWP Success Rate | Expected Points | Risk Level |
|-------|------------------|-----------------|------------|
| Conservative | 95% | 6 + AWP | Low |
| Aggressive | 75% | 22-25 + AWP | Medium-High |

*Note: Parking points (8 or 30) only count at the end of the full match, not during autonomous*

**Recommendation**: Start with conservative routes for consistent AWP, then develop aggressive routes as backup options.

---

## Next Steps

### Immediate Actions
1. **Verify AWP requirements** from latest game manual (v2.1)
2. **Implement odometry system** using existing sensors
3. **Create autonomous selector** for LCD-based route selection
4. **Test basic movement** functions (drive straight, turn, positioning)

### Development Timeline
- **Week 1**: Core movement system + odometry
- **Week 2**: AWP routes implementation + testing
- **Week 3**: Bonus routes + autonomous selector
- **Week 4**: Field testing + refinement

### Testing Protocol
1. **Consistency testing**: 10+ runs per route
2. **AWP verification**: Confirm task completion
3. **Timing validation**: Ensure 15-second completion
4. **Field variations**: Test on different field setups

---

## Questions for Clarification

1. **AWP Tasks**: What are the exact AWP requirements in the current manual?
2. **Park Zone Location**: Where exactly are the red park zones on the field?
3. **Starting Positions**: Precise coordinates for Red Left and Red Right positions?
4. **Field Layout**: Any obstacles or special considerations between start and goals?
5. **Robot Testing**: Do you have access to a practice field for testing?

Once these questions are answered, we can finalize the exact routes and begin implementation!