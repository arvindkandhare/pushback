# Pushback Robot - Controller Key Combinations

## Overview
This document provides a complete reference for all controller button mappings and key combinations for the pushback robot control system.

## 🌟 **New Features**
- **3-Ball Storage Limit**: Automatic storage capacity management (max 3 balls)
- **Color Sorting Integration**: Automatic ball sorting with mid goal ejection
- **Smart Ball Counting**: Only desired color balls count toward storage limit
- **Manual Storage Controls**: Testing controls for storage ball count

## Robot Configuration
- **Drive System**: 6-wheel tank drive (3 wheels per side)
- **Wheels**: 3.75" omni wheels
- **Motors**: 11W motors with green cartridges (18:1 gearing)
- **PTO System**: Pneumatic Power Take-Off for switching middle wheels between drive and scorer modes
- **Intake System**: Pneumatic intake mechanism for collecting balls from tubes
- **Color Sensors**: Dual optical sensors for automatic ball color detection
- **Scoring System**: 
   - **Front**: Left middle motor (via PTO) + pneumatic flap (top goals use intake-assisted storage feed)
   - **Back**: Right middle motor (via PTO) with intake-assisted feed for top goals - *Both motors reversed for proper direction*
  - **Storage**: Maximum 3 balls with automatic counting and color filtering
  - **Color Ejection**: Unwanted balls ejected via back mid goal

---

## 🎮 Controller Layout

### Analog Controls
| Control | Function | Description |
|---------|----------|-------------|
| **Left Analog Y** | Left Tank Drive | Controls left side wheels |
| **Right Analog Y** | Right Tank Drive | Controls right side wheels |

### Digital Buttons

#### 🚗 Drivetrain & PTO Controls
| Button | Function | Description |
|--------|----------|-------------|
| **UP** | PTO Toggle | Switch between drivetrain mode (3-wheel) and scorer mode (2-wheel) |

#### 🍯 Intake Mechanism Controls
| Button | Function | Description |
|--------|----------|-------------|
| **DOWN** | Intake Toggle | **TOGGLE**: Press to extend/retract intake mechanism for ball collection |

#### 🎯 Front Flap Controls
| Button | Function | Description |
|--------|----------|-------------|
| **RIGHT** | Front Flap Toggle | **TOGGLE**: Press to open/close front scoring flap (independent of scoring modes) |

#### ⚽ New Two-Step Scoring System

**Step 1: Mode Selection**
| Button | Function | Description |
|--------|----------|-------------|
| **Y** | Collection Mode | Collection/intake mode (auto-starts intake motor) |
| **A** | Mid Goal | Select mid-level scoring |
| **B** | Immediate Scoring | Immediate scoring from intake (auto-starts intake motor) |
| **X** | Top Goal | Select top-level scoring |

**Step 2: Execution (Toggle)**
| Button | Function | Description |
|--------|----------|-------------|
| **R1** | Back Execute/Stop | **TOGGLE**: Press to start back execution, press again to stop |
| **R2** | Front Execute/Stop | **TOGGLE**: Press to start front execution, press again to stop |

#### 🔧 Testing Controls
| Button | Function | Description |
|--------|----------|-------------|
| **L1** | Test Left Indexer | Cycle through: Forward → Reverse → Stop |
| **L2** | Test Right Indexer | Cycle through: Forward → Reverse → Stop |

---

## 🔄 Operating Modes

### Drive Mode (PTO Extended)
- **Status**: 3-wheel drive per side
- **Middle Wheels**: Connected to drivetrain
- **Use**: Normal driving with maximum traction
- **Toggle**: Press **UP** to switch to Scorer Mode

### Scorer Mode (PTO Retracted)
- **Status**: 2-wheel drive per side
- **Middle Wheels**: Connected to scoring mechanism
- **Use**: Scoring operations and ball handling
- **Toggle**: Press **UP** to switch to Drive Mode

## 🍯 Intake Mechanism

### Pneumatic Intake System
- **Purpose**: Helps collect balls from intake tubes
- **Default State**: **RETRACTED** - intake mechanism is stored
- **Extended State**: **EXTENDED** - intake mechanism is deployed for ball collection
- **Control**: Press **DOWN** button to toggle between retracted and extended
- **Feedback**: Controller rumbles with ".." pattern when toggled

### Intake Operation:
| State | Description | Use Case |
|-------|-------------|----------|
| **Retracted (Default)** | Intake stored/out of the way | Normal driving and maneuvering |
| **Extended** | Intake deployed for collection | Collecting balls from intake tubes |

## 🔧 Front Scoring Mechanism

### Pneumatic Flap System
- **Purpose**: Controls ball release for front scoring
- **Default State**: **CLOSED** - balls are held against the flap
- **During Front Scoring**: **OPENS** automatically to release balls
- **After Scoring**: **CLOSES** automatically to hold new balls

### Key Differences:
| Direction | Mechanism | Behavior |
|-----------|-----------|----------|
| **Front (R2)** | Pneumatic flap control | Opens flap → score → closes flap |
| **Back (R1)** | Direct scoring | No flap - direct ball release |

---

## 🎨 Color Sorting System

### Automatic Ball Sorting
- **Purpose**: Keep only desired color balls, eject unwanted ones via **BACK MID GOAL**
- **Sensors**: Two optical sensors detect ball color and direction
- **Integration**: Works with 3-ball storage limit system

### Color Mode Selection
| Control | Mode | Description |
|---------|------|-------------|
| **Left Stick X (Left)** | COLLECT_RED | Keep red balls, eject blue balls |
| **Right Stick X (Right)** | COLLECT_BLUE | Keep blue balls, eject red balls |
| **RIGHT Arrow** | Toggle ON/OFF | Enable/disable color sorting |

### Color Sorting Controls
| Button | Function | Description |
|--------|----------|-------------|
| **LEFT Arrow** | Manual Eject | Force ejection for testing |
| **L1** | Increase Duration | +50ms ejection time |
| **L2** | Decrease Duration | -50ms ejection time |

### Color Sorting Behavior:
- **Desired Color**: Ball passes through → counted toward storage (up to 3)
- **Unwanted Color**: Ball detected → ejected via back mid goal (no storage impact)
- **Storage Integration**: Only kept balls count toward 3-ball limit

## 💾 Storage Management System

### 3-Ball Storage Limit
- **Capacity**: Maximum 3 balls in storage at any time
- **Display**: Controller shows current count (e.g., "Storage: 2/3")
- **Protection**: System prevents overflow with "STORAGE FULL!" warning

### Manual Storage Controls (Testing)
| Combination | Function | Description |
|-------------|----------|-------------|
| **LEFT + R1** | Add Ball Count | Manually increase storage count |
| **LEFT + R2** | Remove Ball Count | Manually decrease storage count |
| **LEFT (alone)** | Toggle Storage Mode | Enable/disable scoring from storage |

### Storage Behavior:
- **Normal Intake**: Balls count toward storage until 3/3
- **Storage Full**: Further storage attempts blocked with warning
- **Color Sorting**: Only desired color balls count toward limit
- **Scoring from Storage**: Ball count decreases when scoring

---

## 📋 Scoring Workflow

### Step-by-Step Scoring Process:

1. **Switch to Scorer Mode (Optional)**
   - Press **UP** to engage PTO scorer mode if needed
   - LCD will show: "PTO Mode: Scorer (2-wheel drive)"

2. **Deploy Intake (if needed)**
   - Press **DOWN** to extend intake mechanism for ball collection
   - Controller will rumble with ".." pattern
   - Console will show: "Intake: EXTENDED (deployed for collection)"

3. **Select Scoring Mode**
   - Press **Y** for Collection mode (auto-starts intake motor)
   - Press **A** for Mid Goal scoring
   - Press **B** for Immediate scoring (auto-starts intake motor)
   - Press **X** for Top Goal scoring
   - Controller will rumble and display selection

4. **Execute Scoring (Toggle Control)**
   - Press **R1** to START back execution (direct scoring)
   - Press **R1 again** to STOP back execution
   - Press **R2** to START front execution (opens flap → scores → closes flap)  
   - Press **R2 again** to STOP front execution
   - Controller rumbles: Double for start ("..")，Triple for stop ("---")
   - Front flap automatically opens/closes for front scoring

5. **Retract Intake (when done)**
   - Press **DOWN** to retract intake mechanism back to stored position
   - Controller will rumble with ".." pattern
   - Console will show: "Intake: RETRACTED (stored position)"

### Quick Reference Combinations:
| Mode | Direction | Buttons | Result |
|------|-----------|---------|---------|
| Collection | Front | Y → R2 (toggle) | START/STOP left middle + intake |
| Collection | Back | Y → R1 (toggle) | START/STOP right middle + intake |
| Mid Goal | Front | A → R2 (toggle) | START/STOP left middle + flap |
| Mid Goal | Back | A → R1 (toggle) | START/STOP right middle scoring |
| Immediate | Front | B → R2 (toggle) | START/STOP left middle + intake |
| Immediate | Back | B → R1 (toggle) | START/STOP right middle + intake |
| Top Goal | Front | X → R2 (toggle) | START/STOP storage feed (intake + left middle) |
| Top Goal | Back | X → R1 (toggle) | START/STOP storage feed + right middle |

### Intake Operation:
| Function | Button | Result |
|----------|--------|---------|
| Deploy Intake | DOWN | Extend intake mechanism for ball collection |
| Store Intake | DOWN (again) | Retract intake mechanism to stored position |

### Front Flap Operation:
| Function | Button | Result |
|----------|--------|---------|
| Deploy Front Flap | RIGHT | Open front flap for scoring (independent control) |
| Close Front Flap | RIGHT (again) | Close front flap to hold balls |

---

## 🔧 Testing & Diagnostics

### Individual Motor Testing:
- **L1 Button**: Test left middle wheel
  - 1st press: Forward at 100 RPM
  - 2nd press: Reverse at 100 RPM  
  - 3rd press: Stop
- **L2 Button**: Test right middle wheel
  - 1st press: Forward at 100 RPM
  - 2nd press: Reverse at 100 RPM
  - 3rd press: Stop

### System Feedback:
- **Controller Rumble**: Confirms button presses
  - PTO toggle: "." (single pulse)
  - Intake toggle: ".." (double pulse)
  - Scoring start: ".." (double pulse)
  - Scoring stop: "---" (triple pulse)
- **LCD Display**: Shows current mode and status
- **Controller Screen**: Displays current selections
- **Terminal Output**: Debug information (development mode)

---

## ⚠️ Important Notes

### Safety & Operation:
1. **Always check PTO mode** before attempting scoring operations
2. **Scoring requires both direction AND level** to be selected
3. **Input motor has 5-second timeout** for safety
4. **Scoring sequences run for 2 seconds** automatically
5. **Middle wheels are controlled by different systems** based on PTO state
6. **Intake mechanism starts retracted** for safety and clearance

### Troubleshooting:
- **Motors not responding**: Check PTO mode with UP button
- **Scoring not working**: Ensure mode is selected (Y/A/B/X) before execution (R1/R2)
- **Drive issues**: Verify joystick deadzone (>10) and PTO mode
- **Intake not working**: Check pneumatic connections and ADI port D

### Motor Speeds:
- **Top Goal Scoring**: 150 RPM
- **Mid Goal Scoring**: 100 RPM
- **Collection/Immediate**: 120 RPM
- **Testing**: 100 RPM (forward/reverse)

---

## 📁 File Structure
```
pushback_official/
├── src/
│   ├── main.cpp          # Main control loop and opcontrol
│   ├── drivetrain.cpp    # Tank drive and PTO motor control
│   ├── pto.cpp           # Pneumatic system control
│   ├── intake.cpp        # Intake mechanism pneumatic control
│   └── indexer.cpp       # Scoring and intake system
├── include/
│   ├── config.h          # Button mappings and constants
│   ├── drivetrain.h      # Drivetrain class definition
│   ├── pto.h            # PTO class definition
│   ├── intake.h         # Intake class definition
│   └── indexer.h        # IndexerSystem class definition
└── README.md            # This file
```

---

## 🔗 Button Mapping Configuration

All button mappings are defined in `include/config.h` and can be easily modified:

```cpp
// Tank drive control mapping
#define TANK_DRIVE_LEFT_STICK   pros::E_CONTROLLER_ANALOG_LEFT_Y
#define TANK_DRIVE_RIGHT_STICK  pros::E_CONTROLLER_ANALOG_RIGHT_Y

// NEW CONTROL SCHEME: Two-step scoring system
// Step 1: Mode selection buttons (Y/A/B/X)
#define COLLECTION_MODE_BUTTON     pros::E_CONTROLLER_DIGITAL_Y   // Collection/intake mode
#define MID_GOAL_BUTTON           pros::E_CONTROLLER_DIGITAL_A   // Mid level scoring
#define IMMEDIATE_SCORING_BUTTON   pros::E_CONTROLLER_DIGITAL_B   // Immediate scoring from intake
#define TOP_GOAL_BUTTON           pros::E_CONTROLLER_DIGITAL_X   // Top level scoring

// Step 2: Execution buttons (R1/R2)
#define BACK_EXECUTE_BUTTON       pros::E_CONTROLLER_DIGITAL_R1  // Execute selected mode - back
#define FRONT_EXECUTE_BUTTON      pros::E_CONTROLLER_DIGITAL_R2  // Execute selected mode - front

// Additional controls for testing individual indexers
#define LEFT_INDEXER_TEST_BUTTON  pros::E_CONTROLLER_DIGITAL_L1   // Test left indexer
#define RIGHT_INDEXER_TEST_BUTTON pros::E_CONTROLLER_DIGITAL_L2   // Test right indexer

// PTO control (if still needed) - moved to UP button
#define PTO_TOGGLE_BUTTON         pros::E_CONTROLLER_DIGITAL_UP   // PTO toggle (optional)

// Intake mechanism control - DOWN button
#define INTAKE_TOGGLE_BUTTON      pros::E_CONTROLLER_DIGITAL_DOWN // Intake toggle (extend/retract)

// Front flap direct control - RIGHT button
#define FRONT_FLAP_TOGGLE_BUTTON  pros::E_CONTROLLER_DIGITAL_RIGHT // Toggle front flap open/closed
```

### Pneumatic Configuration:
```cpp
// Pneumatic ports (ADI)
#define PTO_LEFT_PNEUMATIC      'A'  // ADI port A
#define PTO_RIGHT_PNEUMATIC     'B'  // ADI port B
#define FRONT_FLAP_PNEUMATIC    'C'  // ADI port C
#define INTAKE_PNEUMATIC        'D'  // ADI port D - NEW

// Intake pneumatic states
#define INTAKE_EXTENDED   true   // Extended = intake mechanism deployed
#define INTAKE_RETRACTED  false  // Retracted = intake mechanism stored (default)
```

---

*Last Updated: October 18, 2025*  
*PROS Version: 4.2.1*  
*Robot: Pushback Official*