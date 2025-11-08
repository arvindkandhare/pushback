# Storage Roller Movement Testing Guide

## Overview
This guide helps test all scenarios where storage rollers should move correctly when scoring from storage.

## Fixed Issues
1. **Front Top Goal Storage Logic**: Fixed front top goal to ignore storage mode since ball is already at front top position
2. **PTO Timing**: Increased pneumatic actuation delay from 50ms to 200ms  
3. **PTO Verification**: Added verification function to ensure PTO switches to scorer mode
4. **Error Handling**: Added error feedback if PTO fails to switch modes

## Test Scenarios

### **Scenario 1: Basic Storage Operation**
**Steps:**
1. Press Y/A/B/X (any scoring mode) to start intake and storage
2. Observe all rollers moving balls to top storage area

**Expected Behavior:**
- Intake motor: Forward (collecting balls)
- Front roller (Left): -275 RPM (half of -550)
- Back roller (Right): -350 RPM helper speed  
- Top roller: 60 RPM (moderate speed to storage)
- Front flap: Closed
- PTO: Scorer mode

**Debug Output:**
```
DEBUG: startIntakeAndStorage() - Face button pressed!
DEBUG: ✅ PTO successfully switched to scorer mode
DEBUG: ✅ Intake motor started
DEBUG: ✅ Front flap closed for storage
DEBUG: ✅ Top indexer moving balls to storage
```

### **Scenario 2: Front Collection from Storage**
**Steps:**
1. Toggle storage mode ON (Left arrow button)
2. Press Y (Collection mode)
3. Press R2 (Front execution)

**Expected Behavior:**
- Front roller (Left): -550 RPM (moving balls from storage)
- Top roller: 200 RPM (storage to front speed)
- Back roller (Right): -350 RPM (normal collection)
- Intake motor: Forward

### **Scenario 3: Back Collection from Storage**
**Steps:**
1. Toggle storage mode ON (Left arrow button)  
2. Press Y (Collection mode)
3. Press R1 (Back execution)

**Expected Behavior:**
- Front roller (Left): -550 RPM (moving balls from storage)
- Top roller: -200 RPM (storage to back speed)
- Back roller (Right): -350 RPM (collection speed)
- Intake motor: Forward

### **Scenario 4: Front Low Goal from Storage** ✅ **VERIFIED**
**Steps:**
1. Toggle storage mode ON (Left arrow button)
2. Press B (Low Goal mode)  
3. Press R2 (Front execution)

**Expected Behavior:**
- Front roller (Left): -550 RPM (moving balls from storage)
- Top roller: 200 RPM (storage to front speed)
- Intake motor: REVERSE (essential for low goal scoring mechanism)

**Note:** Intake reverse is required for low goal scoring - this is the correct behavior

### **Scenario 5: Back Low Goal from Storage** ✅ **VERIFIED**
**Steps:**
1. Toggle storage mode ON (Left arrow button)
2. Press B (Low Goal mode)
3. Press R1 (Back execution)

**Expected Behavior:**
- Front roller (Left): -550 RPM (moving balls from storage)
- Top roller: -200 RPM (storage to back speed)
- Intake motor: REVERSE (essential for low goal scoring mechanism)

**Note:** Intake reverse is required for low goal scoring - this is the correct behavior

### **Scenario 6: Front Mid Goal from Storage**
**Steps:**
1. Toggle storage mode ON (Left arrow button)
2. Press A (Mid Goal mode)
3. Press R2 (Front execution)

**Expected Behavior:**
- Front roller (Left): -550 RPM (moving balls from storage)
- Top roller: 200 RPM (storage to front speed)
- Intake motor: Forward

### **Scenario 7: Back Mid Goal from Storage**
**Steps:**
1. Toggle storage mode ON (Left arrow button)
2. Press A (Mid Goal mode)
3. Press R1 (Back execution)

**Expected Behavior:**
- Front roller (Left): -550 RPM (moving balls from storage)
- Top roller: -200 RPM (storage to back speed)
- Back roller (Right): 500 RPM (mid goal scoring)
- Intake motor: Forward

### **Scenario 8: Front Top Goal from Storage** ⚠️ **FIXED**
**Steps:**
1. Toggle storage mode ON (Left arrow button)
2. Press X (Top Goal mode)
3. Press R2 (Front execution)

**Expected Behavior:**
- Storage mode is **IGNORED** (ball is already at front top position)
- Front roller (Left): -350 RPM (top goal speed)
- Top roller: 400 RPM (front scoring speed)
- Back roller (Right): -350 RPM (helper speed)
- Intake motor: Forward

**Previous Issue:** Was trying to move balls from storage toward back goal, which doesn't make sense for front top goal

### **Scenario 9: Back Top Goal from Storage**
**Steps:**
1. Toggle storage mode ON (Left arrow button)
2. Press X (Top Goal mode)
3. Press R1 (Back execution)

**Expected Behavior:**
- Front roller (Left): -550 RPM (moving balls from storage toward back)
- Top roller: -200 RPM (storage to back speed)
- Back roller (Right): -550 RPM (full speed for top goal scoring)
- Intake motor: Forward

### **Scenario 10: PTO Error Handling** ⚠️ **NEW**
**Steps:**
1. Manually set PTO to drivetrain mode
2. Disable PTO pneumatics (if possible)
3. Press any storage mode button

**Expected Behavior:**
- Error message on controller: "PTO ERROR"
- Controller rumble: "---"
- Storage operation cancelled
- Debug output shows PTO failure

## **Controller Display Indicators**

**Storage Mode Status:**
- `ST*` = Storage mode ON (will score from storage)
- `STo` = Storage mode OFF (will score normally)

**Current Mode Indicators:**
- `C*` = Collection mode active
- `M*` = Mid Goal mode active  
- `L*` = Low Goal mode active
- `T*` = Top Goal mode active

## **Troubleshooting**

### **Storage Rollers Not Moving**
1. Check PTO is in scorer mode
2. Verify storage mode is toggled ON (`ST*`)
3. Check for error messages on controller
4. Ensure mode is selected before pressing R1/R2

### **Conflicting Ball Flow** 
1. Verify intake motor direction in storage scenarios
2. Check roller speed signs (+ vs -)
3. Look for "conflict" warnings in debug output

### **Timing Issues**
1. Allow 200ms after PTO switching
2. Check pneumatic air pressure
3. Verify PTO pneumatics are working

## **Testing Checklist**

- [ ] Scenario 1: Basic Storage Operation
- [ ] Scenario 2: Front Collection from Storage  
- [ ] Scenario 3: Back Collection from Storage
- [ ] Scenario 4: Front Low Goal from Storage (FIXED)
- [ ] Scenario 5: Back Low Goal from Storage (FIXED)
- [ ] Scenario 6: Front Mid Goal from Storage
- [ ] Scenario 7: Back Mid Goal from Storage
- [ ] Scenario 8: Front Top Goal from Storage
- [ ] Scenario 9: Back Top Goal from Storage
- [ ] Scenario 10: PTO Error Handling (NEW)

## **Expected Test Results**

All scenarios should now work correctly with the following improvements:
1. Front top goal now correctly ignores storage mode (ball already at front top position)
2. Proper PTO verification before storage operations
3. Better error handling and user feedback
4. Consistent roller speeds and directions
5. Low goal correctly uses intake reverse (essential for scoring mechanism)