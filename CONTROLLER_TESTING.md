# Controller-Based Testing Guide

## 🎮 Quick Testing with Controller

Since your LCD isn't accessible, you can use the controller for autonomous selection and testing!

### 🔧 Controller Button Mapping for Testing:

**Mode Selection (in disabled/competition_initialize):**
- **UP/DOWN or LEFT/RIGHT**: Change autonomous mode
- **A Button**: Confirm selection
- **A Button (when confirmed)**: Deselect to change mode

**Controller Screen Display:**
- Line 1: Current selected mode
- Line 2: Instructions (UP/DOWN: change, A: confirm)

---

## 🚀 Quick Testing Steps:

### **Step 1: Power On & Connect**
1. Power on robot (keep still for gyro calibration)
2. Connect controller
3. Look at controller screen for mode selection

### **Step 2: Select Test Mode**
1. Use **UP/DOWN arrows** to navigate through modes:
   - DISABLED
   - Red Left AWP
   - Red Left Bonus
   - Red Right AWP
   - Red Right Bonus
   - Skills
   - **Test: Drive** ← Start here
   - Test: Turn
   - Test: Navigation
   - Test: Odometry

2. Press **A** to confirm selection
3. Controller will show "CONFIRMED: Test: Drive"

### **Step 3: Run Test**
1. Place robot in open area (4+ feet of space)
2. **Enable autonomous mode** (competition switch or run autonomous())
3. Robot will execute the test automatically
4. Watch console output for results

### **Step 4: Read Results**
Check the terminal/console for output like:
```
=== STRAIGHT DRIVE TEST ===
Target distance: 24.00 inches
Starting position: (0.00, 0.00, 0.00°)
=== RESULTS ===
Target: 24.00 inches
Actual: 25.30 inches
Error: 1.30 inches (5.4%)
Heading drift: -2.10 degrees
Time taken: 3200 ms
❌ FAIL: Needs calibration
   Adjust TRACKING_WHEEL_DIAMETER in config.h
```

---

## 🛠️ Testing Sequence:

### **Test 1: Drive Accuracy**
1. Select "Test: Drive"
2. Robot drives straight 24 inches
3. **Good result**: < 1" error, < 3° drift
4. **If fails**: Adjust `TRACKING_WHEEL_DIAMETER` in config.h

### **Test 2: Turn Accuracy** 
1. Select "Test: Turn"
2. Robot turns 90 degrees in place
3. **Good result**: < 2° error
4. **If fails**: Adjust turn PID values

### **Test 3: Navigation**
1. Select "Test: Navigation"
2. Robot drives in square pattern and returns to start
3. **Good result**: < 3" from starting position
4. **If fails**: Check encoder mounting

### **Test 4: Odometry Verification**
1. Mark robot's starting position with tape
2. Select "Test: Odometry"
3. Press **A** when prompted to start
4. Robot drives complex pattern and returns
5. **Manually measure** distance from starting mark

---

## 🎯 Quick Calibration Adjustments:

### **Robot drives wrong distance:**
```cpp
// In config.h - adjust this value:
#define TRACKING_WHEEL_DIAMETER  2.75  

// Too far → decrease diameter
// Too short → increase diameter
```

### **Robot oscillates or overshoots:**
```cpp
// In config.h - reduce these values:
#define DRIVE_KP                0.6    // Was 0.8
#define TURN_KP                 1.0    // Was 1.2
```

### **Robot stops short of target:**
```cpp
// In config.h - increase these values:
#define DRIVE_KI                0.02   // Was 0.01
#define TURN_KI                 0.03   // Was 0.02
```

---

## 📱 Controller Display Examples:

**Selecting Mode:**
```
Select: Test: Drive
UP/DOWN: change, A: confirm
```

**Mode Confirmed:**
```
CONFIRMED: Test: Drive
Press A to change
```

**During Test:**
```
Test running...
Check console output
```

---

## 🔄 Change Modes Anytime:

1. **If mode confirmed**: Press **A** to deselect
2. **Use UP/DOWN** to change selection
3. **Press A** to confirm new mode
4. **Run autonomous** to execute test

---

## 📊 Expected Test Results:

| Test | Duration | Good Result |
|------|----------|-------------|
| Drive | ~3 seconds | < 1" error |
| Turn | ~2 seconds | < 2° error |
| Navigation | ~15 seconds | < 3" return error |
| Odometry | ~20 seconds | Manual verification |

---

## 🚨 Troubleshooting:

**Controller not showing selector:**
- Check controller connection
- Make sure robot is in disabled() mode

**Tests not running:**
- Ensure mode is confirmed (shows "CONFIRMED")
- Check autonomous mode is enabled

**Console output not visible:**
- Use PROS terminal or VEX coding studio terminal
- Connect robot via USB cable

**Robot not moving:**
- Check battery level
- Verify motor connections
- Test drivetrain in opcontrol first

---

## 🎮 Driver Control Testing

### **Testing Pneumatic Systems:**

**PTO System Testing:**
1. Enter driver control mode
2. Press **UP** button - robot should switch between drive/scorer modes
3. Controller rumbles with "." pattern on successful toggle
4. Watch for PTO status on controller display

**Intake Testing:**
1. Press **DOWN** button - intake should extend/retract
2. Controller rumbles with ".." pattern on successful toggle
3. Listen for pneumatic "hiss" sound

**Front Flap Testing:**
1. Press **RIGHT** button - front flap should open/close
2. Controller rumbles with "..." pattern (triple rumble)
3. Visually confirm flap movement
4. Test independently - works without scoring mode selection

**Testing Individual Indexers:**
1. **L1** button: Cycles left indexer (Forward → Reverse → Stop)
2. **L2** button: Cycles right indexer (Forward → Reverse → Stop)
3. Each press advances to next state with rumble feedback

---

## ✅ Quick Start Checklist:

- [ ] Power on robot (stationary)
- [ ] Connect controller
- [ ] Select "Test: Drive" with UP/DOWN
- [ ] Press A to confirm
- [ ] Place robot in open space
- [ ] Enable autonomous
- [ ] Check console for results
- [ ] Adjust config.h if needed

**Ready to test!** Start with "Test: Drive" and work through each test mode. The controller will guide you through the selection process! 🎮