# VEX Push Back - Autonomous Testing & Calibration Guide

## 🧪 Testing and Calibration Steps

### Phase 1: Initial Setup & Compilation

#### Step 1: Build and Deploy
```bash
# In your project directory
pros make
pros upload
```

**Expected Output:**
- ✅ Clean compilation (no errors)
- ✅ Successful upload to robot
- ❌ If errors occur, check include paths and sensor port definitions

#### Step 2: Basic System Check
1. **Power on robot** and connect controller
2. **Check LCD display** - should show initialization messages
3. **Watch console output** for:
   ```
   Robot initializing...
   Initializing Autonomous System...
   Calibrating Gyro...
   Gyro ready
   Autonomous System initialized
   ```

---

### Phase 2: Sensor Verification

#### Step 3: Gyro Calibration Test
**Important:** Robot must be completely still during calibration!

1. **Place robot on stable surface** (not moving)
2. **Power on and wait** for gyro calibration (up to 3 seconds)
3. **Check for warnings:**
   - ✅ "Gyro calibration complete"
   - ❌ "WARNING: Gyro calibration timeout!" - Try again

**Troubleshooting:**
- Robot moved during calibration → Power cycle and try again
- Persistent timeout → Check gyro connection on port 13

#### Step 4: Encoder Connection Test
Create a simple test in `opcontrol()` to verify encoder readings:

```cpp
// Add to opcontrol() loop for testing (temporary)
if (counter % 50 == 0) {  // Every second
    printf("Encoders - Vertical: %d, Horizontal: %d\n", 
           vertical_encoder.get_value(), 
           horizontal_encoder.get_value());
}
```

**Test Procedure:**
1. **Push robot forward/backward** - Vertical encoder should change
2. **Push robot left/right** - Horizontal encoder should change
3. **Both encoders reset** when you call `vertical_encoder.reset()`

**Expected Values:**
- Stationary robot: Both encoders ~0
- Moving robot: Encoders show increasing/decreasing values

---

### Phase 3: Basic Movement Calibration

#### Step 5: Straight Line Test
**Goal:** Verify robot drives straight and odometry tracks correctly

**Test Code (add to autonomous function temporarily):**
```cpp
void autonomous() {
    printf("=== STRAIGHT LINE TEST ===\n");
    
    // Set starting position
    autonomous_system.setPosition(0, 0, 0);
    
    // Drive forward 24 inches
    printf("Driving forward 24 inches...\n");
    autonomous_system.driveDistance(24, 0);  // 24" forward, 0° heading
    
    // Check final position
    Position final_pos = autonomous_system.getPosition();
    printf("Final position: (%.2f, %.2f, %.2f°)\n", 
           final_pos.x, final_pos.y, final_pos.heading);
    
    // Expected: x ≈ 24, y ≈ 0, heading ≈ 0
}
```

**Calibration Steps:**
1. **Mark starting position** with tape
2. **Mark 24" ahead** with tape
3. **Run autonomous test**
4. **Measure actual distance traveled**

**Calibration Adjustments:**
If robot travels wrong distance, adjust in `config.h`:
```cpp
// If robot travels too far, decrease diameter
// If robot travels too short, increase diameter
#define TRACKING_WHEEL_DIAMETER  2.75  // Adjust this value
```

#### Step 6: Turn Test
**Goal:** Verify robot turns accurately

**Test Code:**
```cpp
void autonomous() {
    printf("=== TURN TEST ===\n");
    
    autonomous_system.setPosition(0, 0, 0);
    
    // Turn 90 degrees
    printf("Turning 90 degrees...\n");
    autonomous_system.turnToHeading(90);
    
    Position final_pos = autonomous_system.getPosition();
    printf("Final heading: %.2f° (expected: 90°)\n", final_pos.heading);
}
```

**Calibration:**
- **Accurate turn:** Heading should be 90° ± 2°
- **Overshooting:** Decrease turn PID gains
- **Undershooting:** Increase turn PID gains

---

### Phase 4: PID Tuning

#### Step 7: Drive PID Tuning
**Symptoms requiring tuning:**
- **Oscillation:** Robot wobbles side-to-side → Decrease `DRIVE_KD`
- **Slow approach:** Takes too long to reach target → Increase `DRIVE_KP`
- **Overshoot:** Robot goes past target → Decrease `DRIVE_KP` or increase `DRIVE_KD`
- **Never reaches target:** Robot stops short → Increase `DRIVE_KI`

**Tuning in `config.h`:**
```cpp
// Start with these values, then adjust:
#define DRIVE_KP                0.8    // Primary control
#define DRIVE_KI                0.01   // Eliminates steady-state error
#define DRIVE_KD                0.1    // Reduces oscillation
```

#### Step 8: Turn PID Tuning
**Similar process for turning:**
```cpp
#define TURN_KP                 1.2    // Primary turn control
#define TURN_KI                 0.02   // Eliminates heading drift
#define TURN_KD                 0.15   // Reduces turn oscillation
```

---

### Phase 5: Advanced Movement Testing

#### Step 9: Point-to-Point Navigation
**Test Code:**
```cpp
void autonomous() {
    printf("=== NAVIGATION TEST ===\n");
    
    autonomous_system.setPosition(0, 0, 0);
    
    // Drive to several points in sequence
    autonomous_system.driveToPoint(24, 0);   // Forward 24"
    pros::delay(1000);
    
    autonomous_system.driveToPoint(24, 24);  // Right 24"
    pros::delay(1000);
    
    autonomous_system.driveToPoint(0, 24);   // Back to left
    pros::delay(1000);
    
    autonomous_system.driveToPoint(0, 0);    // Return to start
    
    Position final_pos = autonomous_system.getPosition();
    printf("Return accuracy: (%.2f, %.2f) - should be near (0, 0)\n", 
           final_pos.x, final_pos.y);
}
```

**Success Criteria:**
- Robot should end within 2-3" of starting position
- Path should be reasonably straight between points

#### Step 10: Autonomous Selector Test
**Test the LCD selector system:**

1. **Enable competition mode** or run in `disabled()`
2. **Use LCD buttons** to navigate:
   - **LEFT/RIGHT:** Change selection
   - **CENTER:** Confirm selection
3. **Verify modes display correctly:**
   - DISABLED
   - Red Left AWP
   - Red Left Bonus
   - Red Right AWP
   - Red Right Bonus
   - Skills

---

### Phase 6: Integration Testing

#### Step 11: Scoring System Integration
**Test autonomous with your existing scoring system:**

```cpp
void autonomous() {
    printf("=== SCORING INTEGRATION TEST ===\n");
    
    // Test PTO switching
    pto_system->setScorerMode();
    pros::delay(500);
    
    // Test indexer operation
    indexer_system->setMidGoalMode();
    indexer_system->executeFront();
    pros::delay(2000);
    indexer_system->stopAll();
    
    // Test combined movement + scoring
    autonomous_system.driveToPoint(24, 12);
    // Score here
    autonomous_system.driveToPoint(12, 12);
}
```

#### Step 12: Timing Validation
**Ensure autonomous completes within 15 seconds:**

```cpp
void autonomous() {
    uint32_t start_time = pros::millis();
    
    // Your autonomous routine here
    executeRedLeftAWP();
    
    uint32_t total_time = pros::millis() - start_time;
    printf("Autonomous completed in %dms (limit: 15000ms)\n", total_time);
}
```

---

### Phase 7: Field Testing

#### Step 13: Practice Field Validation
**On actual field or field tiles:**

1. **Measure and mark key positions:**
   - Starting positions (Red Left, Red Right)
   - Goal locations
   - AWP task locations

2. **Test consistency:**
   - Run same route 5+ times
   - Measure ending position accuracy
   - Note any consistent drift or errors

3. **Environmental factors:**
   - Different battery levels
   - Field tile variations
   - Lighting conditions

---

## 🔧 Common Issues & Solutions

### Compilation Errors
```bash
# Missing includes
error: 'AutonomousSystem' was not declared
```
**Solution:** Check all `#include` statements in main.cpp

### Runtime Issues
```
Gyro calibration timeout!
```
**Solution:** Ensure robot is stationary during power-on

### Odometry Drift
```
Robot ends up far from expected position
```
**Solutions:**
1. Check encoder wheel contact with ground
2. Verify `TRACKING_WHEEL_DIAMETER` setting
3. Ensure encoders aren't slipping

### Movement Issues
```
Robot doesn't move or moves erratically
```
**Solutions:**
1. Check PID gain values
2. Verify motor connections and directions
3. Test drivetrain in opcontrol first

---

## 📊 Calibration Values to Record

Create a calibration log with these measured values:

```
=== CALIBRATION LOG ===
Date: ___________
Battery Level: ___________

Tracking Wheel Diameter: ___________
Drive Distance (24" test): Actual _____ Expected 24"
Turn Angle (90° test): Actual _____ Expected 90°

PID Values:
DRIVE_KP: _____  DRIVE_KI: _____  DRIVE_KD: _____
TURN_KP: _____   TURN_KI: _____   TURN_KD: _____

Position Accuracy (return to start):
X Error: _____ inches
Y Error: _____ inches

Autonomous Timing:
Red Left AWP: _____ ms
Red Right AWP: _____ ms
```

---

## 🚀 Next Steps After Calibration

Once calibration is complete:

1. **Implement specific AWP routes** based on game manual requirements
2. **Add field position constants** for goals and key locations
3. **Create competition-ready autonomous routines**
4. **Test with alliance partner coordination**

Ready to start testing? Begin with **Phase 1: Initial Setup & Compilation**! 🎯