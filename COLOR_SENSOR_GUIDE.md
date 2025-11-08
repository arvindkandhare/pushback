# Color Sensing and Sorting System

## Overview

The color sensing and sorting system automatically detects ball colors using two V5 optical sensors and ejects undesired colored balls through a pneumatic mechanism positioned at the back mid out location.

## Hardware Configuration

### Sensors
- **Color Sensor 1** (Port 5): Entry detection sensor - first sensor to detect incoming balls
- **Color Sensor 2** (Port 11): Confirmation sensor - validates color and tracks ball direction

### Pneumatics
- **Ball Ejection Solenoid** (ADI Port C): Pneumatic cylinder that pushes balls out from the back mid position

### Sensor Placement
The sensors should be positioned:
1. **Sensor 1**: At the ball intake entry point
2. **Sensor 2**: Further along the ball path, near the ejection mechanism
3. **Ejection Mechanism**: Positioned to push balls out from the back mid area

## Controller Controls

### Color Mode Selection
- **Left Analog Stick X (Left)**: Set to collect RED balls (eject blue balls)
- **Right Analog Stick X (Right)**: Set to collect BLUE balls (eject red balls)

### Manual Controls
- **LEFT Button**: Manual ball ejection trigger (for testing)
- **RIGHT Button**: Toggle sorting on/off (overrides normal operation)

### Control Feedback
- Controller rumble patterns indicate mode changes
- Controller display shows current sorting mode
- Console output provides detailed status information

## Sorting Modes

### SortingMode::COLLECT_RED
- **Keep**: Red balls
- **Eject**: Blue balls
- **Use Case**: Red alliance, collecting red balls for scoring

### SortingMode::COLLECT_BLUE  
- **Keep**: Blue balls
- **Eject**: Red balls
- **Use Case**: Blue alliance, collecting blue balls for scoring

### SortingMode::COLLECT_ALL
- **Keep**: All balls
- **Eject**: None
- **Use Case**: Sorting disabled, collect everything

### SortingMode::EJECT_ALL
- **Keep**: None
- **Eject**: All balls
- **Use Case**: Defensive mode, reject all opponent balls

## Detection Algorithm

### Color Detection Process
1. Ball approaches **Sensor 1** (entry point)
2. System performs multiple color readings for confirmation
3. Ball continues to **Sensor 2** (confirmation point)
4. System validates color consistency between sensors
5. Direction of ball movement is determined from sensor timing
6. Decision made whether to eject or keep the ball

### Color Thresholds
- **Red Balls**: Hue 0-30° or 330-360° (wraps around color wheel)
- **Blue Balls**: Hue 200-250°
- **Minimum Saturation**: 50 (ensures vivid colors)
- **Minimum Brightness**: 30 (ensures adequate lighting)

### Confirmation Requirements
- **3 consecutive consistent readings** required for color confirmation
- **Maximum 100ms proximity threshold** for ball presence detection
- **2 second timeout** for ball passage between sensors

## Ball Ejection Mechanism

### Ejection Timing
1. Ball color is confirmed at Sensor 1
2. System waits for ball to reach Sensor 2 area
3. **200ms delay** before ejection (allows ball positioning)
4. **500ms ejection duration** (pneumatic activation time)
5. Automatic retraction of ejection mechanism

### Ejection Location
- Balls are ejected from the **back mid out** position
- This allows the robot to continue collecting while removing unwanted balls
- Ejected balls fall away from the robot and do not interfere with continued operation

## Integration with Robot Systems

### Initialization
```cpp
// Create color sensor system during robot initialization
color_sensor_system = new ColorSensorSystem(BALL_EJECT_PNEUMATIC);

// Initialize the system
if (color_sensor_system->initialize()) {
    printf("✅ Color sensor system ready\n");
}
```

### Main Loop Integration
```cpp
// Update color sensor system in main control loop
color_sensor_system->update();

// Handle controller inputs for mode changes
// (automatically handled in main.cpp opcontrol function)
```

## Status Monitoring

### Real-Time Statistics
- **Red balls detected**: Count of red balls identified
- **Blue balls detected**: Count of blue balls identified  
- **Balls ejected**: Total number of balls ejected
- **False detections**: Count of inconsistent color readings

### Debug Information
```cpp
// Print detailed status
color_sensor_system->printStatus();

// Get statistics programmatically
int red, blue, ejected, false_count;
color_sensor_system->getStatistics(red, blue, ejected, false_count);
```

### Console Output
The system provides detailed console logging:
- Ball detection events
- Color confirmation results
- Direction determination
- Ejection triggers
- Mode changes
- Error conditions

## Testing and Calibration

### Sensor Testing
```cpp
// Test both sensors
bool sensors_ok = color_sensor_system->testSensors();
```

### Manual Testing
- Use LEFT button for manual ejection testing
- Monitor console output for sensor readings
- Verify color detection accuracy with known colored balls
- Test timing between sensors with ball movement

### Calibration Tips
1. **Lighting**: Ensure consistent lighting conditions
2. **Sensor Height**: Position sensors at proper height for ball detection
3. **Timing**: Verify ball travel time between sensors is reasonable
4. **Ejection Alignment**: Ensure ejection mechanism properly pushes balls out

## Troubleshooting

### Common Issues

**No Color Detection**
- Check sensor connections (ports 5 and 11)
- Verify adequate lighting
- Ensure balls are passing close enough to sensors

**Inconsistent Color Readings**
- Check for reflective surfaces near sensors
- Verify sensor mounting stability
- Adjust color thresholds if necessary

**Ejection Not Working**
- Check pneumatic connection (ADI port C)
- Verify solenoid operation
- Check timing delays in code

**False Ejections**
- Reduce sensitivity by increasing confirmation count
- Check for interference from ambient lighting
- Verify color threshold settings

### Performance Optimization
- Adjust `COLOR_CONFIRMATION_COUNT` for speed vs accuracy
- Modify `BALL_EJECT_DELAY_MS` for optimal ejection timing
- Tune color thresholds for specific ball types used in competition

## Competition Strategy

### Alliance Color Setup
1. **Before match**: Set sorting mode to match alliance color
2. **Red Alliance**: Use COLLECT_RED mode
3. **Blue Alliance**: Use COLLECT_BLUE mode
4. **Skills/Practice**: Use COLLECT_ALL mode

### Autonomous Integration
The color sensor system operates independently and can function during both autonomous and driver control periods without interference.

### Match Strategy
- **Early Game**: Set to collect alliance color
- **Mid Game**: Can switch to COLLECT_ALL for maximum ball collection
- **End Game**: Can switch to EJECT_ALL for defensive ball rejection