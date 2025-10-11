# Pushback Robot - Controller Key Combinations

## Overview
This document provides a complete reference for all controller button mappings and key combinations for the pushback robot control system.

## Robot Configuration
- **Drive System**: 6-wheel tank drive (3 wheels per side)
- **Wheels**: 3.75" omni wheels
- **Motors**: 11W motors with green cartridges (18:1 gearing)
- **PTO System**: Pneumatic Power Take-Off for switching middle wheels between drive and scorer modes
- **Scoring System**: Front indexer + back indexer (via PTO middle wheels)

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
| **R1** | PTO Toggle | Switch between drivetrain mode (3-wheel) and scorer mode (2-wheel) |

#### ⚽ Scoring System Controls
| Button | Function | Description |
|--------|----------|-------------|
| **L1** | Front Scoring | Select front scoring direction |
| **L2** | Back Scoring | Select back scoring direction |
| **X** | Long Goal | Select top scoring level (higher goal) |
| **B** | Mid Goal | Select bottom scoring level (lower goal) |
| **A** | Execute Scoring | Start scoring sequence with selected direction & level |
| **R2** | Ball Intake | Hold to run input motor for ball collection |

#### 🔧 Testing Controls
| Button | Function | Description |
|--------|----------|-------------|
| **Y** | Test Left Indexer | Cycle through: Forward → Reverse → Stop |
| **UP** | Test Right Indexer | Cycle through: Forward → Reverse → Stop |

---

## 🔄 Operating Modes

### Drive Mode (PTO Extended)
- **Status**: 3-wheel drive per side
- **Middle Wheels**: Connected to drivetrain
- **Use**: Normal driving with maximum traction
- **Toggle**: Press **R1** to switch to Scorer Mode

### Scorer Mode (PTO Retracted)
- **Status**: 2-wheel drive per side
- **Middle Wheels**: Connected to scoring mechanism
- **Use**: Scoring operations and ball handling
- **Toggle**: Press **R1** to switch to Drive Mode

---

## 📋 Scoring Workflow

### Step-by-Step Scoring Process:

1. **Switch to Scorer Mode**
   - Press **R1** to engage PTO scorer mode
   - LCD will show: "PTO Mode: Scorer (2-wheel drive)"

2. **Select Scoring Direction**
   - Press **L1** for front scoring
   - Press **L2** for back scoring
   - Controller will rumble and display selection

3. **Select Scoring Level**
   - Press **X** for long goal (top scoring)
   - Press **B** for mid goal (bottom scoring)
   - Controller will rumble and display selection

4. **Execute Scoring**
   - Press **A** to start scoring sequence
   - Motors will run for 2 seconds automatically
   - Controller will rumble twice to confirm

5. **Ball Intake (Optional)**
   - Hold **R2** to run intake motor
   - Release to stop intake

### Quick Reference Combinations:
| Direction | Level | Buttons | Result |
|-----------|--------|---------|---------|
| Front | Long Goal | L1 → X → A | Front indexer runs forward |
| Front | Mid Goal | L1 → B → A | Front indexer runs reverse |
| Back | Long Goal | L2 → X → A | Right middle wheel runs forward |
| Back | Mid Goal | L2 → B → A | Right middle wheel runs reverse |

---

## 🔧 Testing & Diagnostics

### Individual Motor Testing:
- **Y Button**: Test left middle wheel
  - 1st press: Forward at 100 RPM
  - 2nd press: Reverse at 100 RPM  
  - 3rd press: Stop
- **UP Button**: Test right middle wheel
  - 1st press: Forward at 100 RPM
  - 2nd press: Reverse at 100 RPM
  - 3rd press: Stop

### System Feedback:
- **Controller Rumble**: Confirms button presses
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

### Troubleshooting:
- **Motors not responding**: Check PTO mode with R1
- **Scoring not working**: Ensure both direction (L1/L2) and level (X/B) are selected
- **Intake not working**: Hold R2 continuously, check for timeout
- **Drive issues**: Verify joystick deadzone (>10) and PTO mode

### Motor Speeds:
- **Long Goal Scoring**: 150 RPM
- **Mid Goal Scoring**: 100 RPM
- **Ball Intake**: 120 RPM
- **Testing**: 100 RPM (forward/reverse)

---

## 📁 File Structure
```
pushback_official/
├── src/
│   ├── main.cpp          # Main control loop and opcontrol
│   ├── drivetrain.cpp    # Tank drive and PTO motor control
│   ├── pto.cpp           # Pneumatic system control
│   └── indexer.cpp       # Scoring and intake system
├── include/
│   ├── config.h          # Button mappings and constants
│   ├── drivetrain.h      # Drivetrain class definition
│   ├── pto.h            # PTO class definition
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

// PTO control button
#define PTO_TOGGLE_BUTTON       pros::E_CONTROLLER_DIGITAL_R1

// Indexer and scoring system controls
#define INPUT_MOTOR_BUTTON      pros::E_CONTROLLER_DIGITAL_R2
#define FRONT_SCORING_BUTTON    pros::E_CONTROLLER_DIGITAL_L1
#define BACK_SCORING_BUTTON     pros::E_CONTROLLER_DIGITAL_L2
#define LONG_GOAL_BUTTON        pros::E_CONTROLLER_DIGITAL_X
#define MID_GOAL_BUTTON         pros::E_CONTROLLER_DIGITAL_B
#define EXECUTE_SCORING_BUTTON  pros::E_CONTROLLER_DIGITAL_A

// Testing controls
#define LEFT_INDEXER_TEST_BUTTON  pros::E_CONTROLLER_DIGITAL_Y
#define RIGHT_INDEXER_TEST_BUTTON pros::E_CONTROLLER_DIGITAL_UP
```

---

*Last Updated: October 11, 2025*  
*PROS Version: 4.2.1*  
*Robot: Pushback Official*