# PID Tuning Guide for VEX Push Back Robot

## 🎯 What is PID and Why Tune It?

**PID (Proportional-Integral-Derivative)** controls how smoothly and accurately your robot moves:

- **P (Proportional)**: How hard to "push" toward the target
- **I (Integral)**: Corrects for consistent errors over time  
- **D (Derivative)**: Reduces oscillation/overshoot

**Poor PID = Shaky, inaccurate movement**  
**Good PID = Smooth, precise movement**

---

## 🚀 Quick Start: Use Default Values First

Your robot already has starting PID values in `config.h`:

```cpp
// Drive PID (for moving to positions)
#define DRIVE_KP                0.8    // Proportional gain
#define DRIVE_KI                0.01   // Integral gain  
#define DRIVE_KD                0.1    // Derivative gain

// Turn PID (for rotating to headings)
#define TURN_KP                 1.2    // Turn proportional
#define TURN_KI                 0.02   // Turn integral
#define TURN_KD                 0.15   // Turn derivative
```

**These should work reasonably well to start!**

---

## 📋 PID Tuning Process

### **Step 1: Test with Defaults**
1. Run "Test: Drive" and "Test: Turn" modes
2. Observe the robot's behavior
3. Note any problems from the list below

### **Step 2: Identify Problems**

**🔍 Drive Problems:**

| Behavior | Problem | Solution |
|----------|---------|----------|
| Robot wobbles side-to-side | Too much D gain | Reduce `DRIVE_KD` |
| Robot oscillates back/forth around target | Too much P gain | Reduce `DRIVE_KP` |
| Robot overshoots target significantly | Too much P gain | Reduce `DRIVE_KP` |
| Robot stops short, never reaches target | Too little I gain | Increase `DRIVE_KI` |
| Robot is very slow to start moving | Too little P gain | Increase `DRIVE_KP` |
| Robot jerky, stops and starts | Too little D gain | Increase `DRIVE_KD` |

**🔄 Turn Problems:**

| Behavior | Problem | Solution |
|----------|---------|----------|
| Robot oscillates left/right around target angle | Too much P gain | Reduce `TURN_KP` |
| Robot overshoots turn significantly | Too much P gain | Reduce `TURN_KP` |
| Robot stops turning before reaching target | Too little I gain | Increase `TURN_KI` |
| Robot turns very slowly | Too little P gain | Increase `TURN_KP` |
| Robot "vibrates" while turning | Too much D gain | Reduce `TURN_KD` |

### **Step 3: Adjust Values Gradually**

**Make small changes** (±0.1 to ±0.3 at a time):

```cpp
// Example: Robot overshoots drive target
#define DRIVE_KP                0.6    // Was 0.8, reduced by 0.2

// Example: Robot oscillates during turns  
#define TURN_KP                 1.0    // Was 1.2, reduced by 0.2
```

### **Step 4: Test After Each Change**
1. Recompile and upload
2. Run the same test again
3. Check if behavior improved
4. Repeat until satisfied

---

## 🛠️ PID Tuning Method

### **Method 1: One Parameter at a Time**

**Start with P gain:**
1. Set I and D to 0
2. Increase P until robot reaches target but oscillates
3. Reduce P slightly until oscillation stops

**Add D gain:**
1. Increase D to reduce any remaining oscillation
2. Too much D makes robot sluggish

**Add I gain:**
1. Add small I value to eliminate steady-state error
2. Too much I causes oscillation

### **Method 2: Use the Test Results**

The built-in tests tell you exactly what to adjust:

```
❌ FAIL: Needs calibration
   Adjust TRACKING_WHEEL_DIAMETER in config.h
```

or for PID issues:

```
Robot overshoots target by 3.2 inches
Reduce DRIVE_KP gain
```

---

## 📊 Good PID Values by Robot Type

**Typical ranges for VEX robots:**

```cpp
// Conservative (smooth, slower)
DRIVE_KP: 0.4-0.8
DRIVE_KI: 0.005-0.02  
DRIVE_KD: 0.05-0.15

// Aggressive (fast, might oscillate)
DRIVE_KP: 0.8-1.5
DRIVE_KI: 0.02-0.05
DRIVE_KD: 0.1-0.3

// Turn values usually higher
TURN_KP: 1.0-2.0
TURN_KI: 0.01-0.05
TURN_KD: 0.1-0.4
```

---

## 🎮 Live PID Tuning (Advanced)

Want to tune PID without recompiling? Add this to your code:

```cpp
// In opcontrol(), add live tuning with controller
if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
    // Increase drive P gain
    DRIVE_KP += 0.1;
    printf("Drive KP now: %.2f\n", DRIVE_KP);
}

if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
    // Decrease drive P gain  
    DRIVE_KP -= 0.1;
    printf("Drive KP now: %.2f\n", DRIVE_KP);
}
```

---

## ✅ How to Know PID is Good

**Good Drive PID:**
- Robot reaches target within 1-2 inches
- No oscillation or wobbling
- Smooth acceleration and deceleration
- Reaches target in reasonable time (2-4 seconds for 24")

**Good Turn PID:**
- Robot reaches target angle within 2-3 degrees
- No back-and-forth oscillation
- Smooth rotation
- Reaches target quickly (1-3 seconds for 90°)

---

## 🚨 Common PID Mistakes

❌ **Making huge changes** (changing 0.8 to 2.0)  
✅ **Make small incremental changes** (0.8 to 0.9)

❌ **Tuning multiple parameters at once**  
✅ **Change one parameter at a time**

❌ **Only testing one movement**  
✅ **Test various distances and angles**

❌ **Ignoring battery level**  
✅ **Test with fresh battery (consistent power)**

---

## 🔧 Quick PID Tuning Workflow

1. **Start with defaults** - they should work "okay"
2. **Run "Test: Drive"** - observe behavior
3. **If robot overshoots**: Reduce `DRIVE_KP` by 0.2
4. **If robot undershoots**: Increase `DRIVE_KP` by 0.2  
5. **If robot oscillates**: Reduce `DRIVE_KP` by 0.1, increase `DRIVE_KD` by 0.05
6. **Repeat until smooth**
7. **Do same process for "Test: Turn"** with turn PID values

**Most robots need 3-5 iterations to get good PID values.**

---

## 💡 Pro Tips

- **Fresh battery** gives most consistent results
- **Smooth field surface** reduces tuning noise
- **Test multiple distances** (12", 24", 36") to ensure consistency
- **Save good values** in comments for future reference
- **Different loads** (carrying blocks) may need different PID

**Ready to tune?** Start with the default values and run your first test! The robot will tell you what needs adjustment. 🎯