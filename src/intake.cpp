/**
 * \file intake.cpp
 *
 * Front match loader system implementation.
 * Manages motor and rotational encoder that controls front match loader deployment for ball collection.
 */

#include "intake.h"
#include <climits>  // For INT_MAX and INT_MIN

Intake::Intake() 
    : front_loader_motor(FRONT_LOADER_MOTOR_PORT),
      front_loader_sensor(FRONT_LOADER_ENCODER_TOP),
      front_loader_deployed(FRONT_LOADER_DEFAULT_STATE),
      front_loader_target_position(FRONT_LOADER_RETRACTED_POSITION),
      last_button_state(false),
      last_l1_button_state(false),
      last_l2_button_state(false),
      sensor_zero_value(0.0) {
    
    // Configure motor
    front_loader_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    front_loader_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    
    // Reverse motor if needed (based on config)
    front_loader_motor.set_reversed(FRONT_LOADER_REVERSE_MOTOR);
    
    // Reset motor's internal encoder 
    front_loader_motor.tare_position();
    
    // Calibrate sensor zero position (current position)
    pros::delay(100);  // Give sensor time to settle
    sensor_zero_value = front_loader_sensor.get_value();
    
    printf("Front Loader: Initialized with motor port %d, sensor port %c\n", 
           FRONT_LOADER_MOTOR_PORT, FRONT_LOADER_ENCODER_TOP);
    printf("  Motor reversed: %s\n", FRONT_LOADER_REVERSE_MOTOR ? "YES" : "NO");
    printf("  Motor internal encoder reset to 0\n");
    printf("  Sensor zero calibrated to: %.0f\n", sensor_zero_value);
    printf("  Position feedback method: %s\n", USE_MOTOR_ENCODER_ONLY ? "MOTOR ENCODER" : "POTENTIOMETER");
    
    if (!USE_MOTOR_ENCODER_ONLY) {
        printf("  Potentiometer mounted on: %s\n", POTENTIOMETER_MOUNTED_ON_MOTOR ? "MOTOR SHAFT" : "LOADER ARM");
        
        // Calculate and display range limitations
        double max_loader_range = POTENTIOMETER_MOUNTED_ON_MOTOR ? 
            POTENTIOMETER_RANGE_DEGREES / FRONT_LOADER_GEAR_RATIO : 
            POTENTIOMETER_RANGE_DEGREES;
        printf("  Maximum loader arm range: ±%.1f degrees\n", max_loader_range / 2.0);
        printf("  Target deploy position: %.1f degrees\n", (double)FRONT_LOADER_DEPLOYED_POSITION);
        
        if (fabs((double)FRONT_LOADER_DEPLOYED_POSITION) > max_loader_range / 2.0) {
            printf("  WARNING: Target position exceeds potentiometer range!\n");
            printf("  Consider switching to motor encoder mode\n");
        }
    } else {
        printf("  Using motor encoder - unlimited range available\n");
        printf("  Target deploy position: %.1f degrees\n", (double)FRONT_LOADER_DEPLOYED_POSITION);
    }
    
    // Check sensor connectivity
    int32_t sensor_test = front_loader_sensor.get_value();
    if (sensor_test < 0 || sensor_test > POTENTIOMETER_MAX_VALUE) {
        printf("  WARNING: Sensor reading out of range (%d)!\n", sensor_test);
    } else {
        printf("  Sensor reading: %d (%.1f%% of range)\n", sensor_test, 
               (double)sensor_test / POTENTIOMETER_MAX_VALUE * 100.0);
    }
    
    // Set initial front loader state
    if (front_loader_deployed == FRONT_LOADER_DEPLOYED) {
        deploy();
    } else {
        retract();
    }
    
    // Print initial debug info
    printDebugInfo();
}

void Intake::retract() {
    // Move to retracted position (0 degrees)
    setPosition(FRONT_LOADER_RETRACTED_POSITION);
    front_loader_deployed = FRONT_LOADER_RETRACTED;
    
    printf("Front Loader: RETRACTING to %.1f degrees\n", (double)FRONT_LOADER_RETRACTED_POSITION);
    printf("  Current position: %.1f degrees (motor: %.1f)\n", getPosition(), getMotorPosition());
}

void Intake::deploy() {
    // Move to deployed position (-110 degrees)  
    setPosition(FRONT_LOADER_DEPLOYED_POSITION);
    front_loader_deployed = FRONT_LOADER_DEPLOYED;
    
    printf("Front Loader: DEPLOYING to %d degrees\n", FRONT_LOADER_DEPLOYED_POSITION);
    printf("  Current position: %.1f degrees (motor: %.1f)\n", getPosition(), getMotorPosition());
}

void Intake::toggle() {
    if (front_loader_deployed == FRONT_LOADER_DEPLOYED) {
        retract();
    } else {
        deploy();
    }
}

bool Intake::isDeployed() const {
    return front_loader_deployed == FRONT_LOADER_DEPLOYED;
}

bool Intake::isRetracted() const {
    return front_loader_deployed == FRONT_LOADER_RETRACTED;
}

double Intake::getPosition() const {
    if (USE_MOTOR_ENCODER_ONLY) {
        // Use motor's internal encoder - reliable and unlimited range
        double motor_degrees = front_loader_motor.get_position();
        return motorDegreesToLoaderDegrees(motor_degrees);
    } else {
        // Use potentiometer with range checking
        int32_t raw_sensor = front_loader_sensor.get_value();
        
        // Check for invalid sensor readings
        if (raw_sensor < 0 || raw_sensor > POTENTIOMETER_MAX_VALUE) {
            printf("WARNING: Invalid sensor reading (%d), using motor encoder instead\n", raw_sensor);
            // Fallback to motor encoder
            double motor_degrees = front_loader_motor.get_position();
            return motorDegreesToLoaderDegrees(motor_degrees);
        }
        
        if (POTENTIOMETER_MOUNTED_ON_MOTOR) {
            // Potentiometer is on motor shaft - convert motor degrees to loader degrees
            double motor_degrees = ((double)(raw_sensor - sensor_zero_value) / POTENTIOMETER_MAX_VALUE) * POTENTIOMETER_RANGE_DEGREES;
            return motorDegreesToLoaderDegrees(motor_degrees);
        } else {
            // Potentiometer is on loader arm - direct reading
            double loader_degrees = ((double)(raw_sensor - sensor_zero_value) / POTENTIOMETER_MAX_VALUE) * POTENTIOMETER_RANGE_DEGREES;
            return loader_degrees;
        }
    }
}

double Intake::getMotorPosition() const {
    if (USE_MOTOR_ENCODER_ONLY) {
        // Use motor's internal encoder
        return front_loader_motor.get_position();
    } else {
        // Use potentiometer with range checking
        int32_t raw_sensor = front_loader_sensor.get_value();
        
        // Check for invalid sensor readings
        if (raw_sensor < 0 || raw_sensor > POTENTIOMETER_MAX_VALUE) {
            // Fallback to motor encoder
            return front_loader_motor.get_position();
        }
        
        if (POTENTIOMETER_MOUNTED_ON_MOTOR) {
            // Potentiometer is on motor shaft - direct reading
            return ((double)(raw_sensor - sensor_zero_value) / POTENTIOMETER_MAX_VALUE) * POTENTIOMETER_RANGE_DEGREES;
        } else {
            // Potentiometer is on loader arm - convert to motor degrees
            double loader_degrees = ((double)(raw_sensor - sensor_zero_value) / POTENTIOMETER_MAX_VALUE) * POTENTIOMETER_RANGE_DEGREES;
            return loaderDegreesToMotorDegrees(loader_degrees);
        }
    }
}

bool Intake::isAtTarget() const {
    double current_position = getPosition();
    double position_error = fabs(current_position - front_loader_target_position);
    return position_error <= FRONT_LOADER_POSITION_TOLERANCE;
}

void Intake::update(pros::Controller& controller) {
    // Get current button states
    bool current_button_state = controller.get_digital(INTAKE_TOGGLE_BUTTON);
    bool current_l1_button_state = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool current_l2_button_state = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    
    // Check for toggle button press (rising edge detection) - resets to original position
    if (current_button_state && !last_button_state) {
        printf("Front Loader: Toggle button pressed! Resetting to original position\n");
        printf("  Before reset - Position: %.1f° (motor: %.1f°)\n", getPosition(), getMotorPosition());
        
        resetToOriginal();
        
        printf("  After reset - Target: %.1f°, State: %s\n", front_loader_target_position, getCurrentStateString());
        
        // Provide haptic feedback - different pattern from PTO
        controller.rumble("..");
    }
    
    // Check for L1 button press (rising edge detection) - adjust +1 degree
    if (current_l1_button_state && !last_l1_button_state) {
        printf("Front Loader: L1 pressed! Adjusting +1 degree\n");
        printf("  Before adjustment - Position: %.1f°, Target: %.1f°\n", getPosition(), front_loader_target_position);
        
        adjustPosition(1.0);
        
        printf("  After adjustment - New Target: %.1f°\n", front_loader_target_position);
        
        // Provide brief haptic feedback
        controller.rumble(".");
    }
    
    // Check for L2 button press (rising edge detection) - adjust -1 degree
    if (current_l2_button_state && !last_l2_button_state) {
        printf("Front Loader: L2 pressed! Adjusting -1 degree\n");
        printf("  Before adjustment - Position: %.1f°, Target: %.1f°\n", getPosition(), front_loader_target_position);
        
        adjustPosition(-1.0);
        
        printf("  After adjustment - New Target: %.1f°\n", front_loader_target_position);
        
        // Provide brief haptic feedback
        controller.rumble(".");
    }
    
    // Update last button states for next iteration
    last_button_state = current_button_state;
    last_l1_button_state = current_l1_button_state;
    last_l2_button_state = current_l2_button_state;
    
    // Continuous position monitoring (every 100ms to avoid spam)
    static uint32_t last_debug_time = 0;
    uint32_t current_time = pros::millis();
    if (current_time - last_debug_time > 100) {
        double current_pos = getPosition();
        double motor_pos = getMotorPosition();
        double error = fabs(current_pos - front_loader_target_position);
        
        // Check for motor stalling
        double motor_current = front_loader_motor.get_current_draw();
        double motor_velocity = front_loader_motor.get_actual_velocity();
        bool motor_stalled = (motor_current > 1500 && fabs(motor_velocity) < 5.0);  // High current, low velocity
        
        // Front loader status logging removed to reduce console spam
        // printf("Front Loader Status: Pos=%.1f° Target=%.1f° Error=%.1f° Motor=%.1f° Current=%dmA Vel=%.1fRPM AtTarget=%s%s\n", 
        //        current_pos, front_loader_target_position, error, motor_pos, 
        //        (int)motor_current, motor_velocity,
        //        isAtTarget() ? "YES" : "NO",
        //        motor_stalled ? " STALLED!" : "");
        
        // If motor is stalled, stop and report
        if (motor_stalled) {
            front_loader_motor.brake();
            printf("WARNING: Motor stalled at %.1f°! Physical obstruction detected.\n", current_pos);
            printf("Consider reducing target angle or checking for mechanical interference.\n");
        }
        
        last_debug_time = current_time;
    }
    
    // Check if we're at target position and stop motor if so
    if (isAtTarget()) {
        front_loader_motor.brake();
    }
}

const char* Intake::getCurrentStateString() const {
    return (front_loader_deployed == FRONT_LOADER_DEPLOYED) ? "Deployed" : "Retracted";
}

void Intake::printDebugInfo() const {
    printf("\n=== FRONT LOADER DEBUG INFO ===\n");
    printf("Hardware Configuration:\n");
    printf("  Motor Port: %d\n", FRONT_LOADER_MOTOR_PORT);
    printf("  Encoder Port: %c\n", FRONT_LOADER_ENCODER_TOP);
    printf("  Gear Ratio: %.1f:1\n", FRONT_LOADER_GEAR_RATIO);
    printf("  Motor Speed: %d RPM\n", FRONT_LOADER_MOTOR_SPEED);
    printf("  Position Tolerance: %d degrees\n", FRONT_LOADER_POSITION_TOLERANCE);
    
    printf("\nPosition Configuration:\n");
    printf("  Retracted Position: %d degrees\n", FRONT_LOADER_RETRACTED_POSITION);
    printf("  Deployed Position: %d degrees\n", FRONT_LOADER_DEPLOYED_POSITION);
    
    printf("\nCurrent Status:\n");
    printf("  State: %s\n", getCurrentStateString());
    printf("  Target Position: %.1f degrees\n", front_loader_target_position);
    printf("  Current Loader Position: %.1f degrees\n", getPosition());
    printf("  Current Motor Position: %.1f degrees\n", getMotorPosition());
    printf("  Raw External Sensor: %d\n", front_loader_sensor.get_value());
    printf("  Motor Internal Encoder: %.1f degrees\n", front_loader_motor.get_position());
    printf("  Position Error: %.1f degrees\n", fabs(getPosition() - front_loader_target_position));
    printf("  At Target: %s\n", isAtTarget() ? "YES" : "NO");
    
    printf("\nMotor Status:\n");
    printf("  Motor Temperature: %.1f°C\n", front_loader_motor.get_temperature());
    printf("  Motor Voltage: %d mV\n", front_loader_motor.get_voltage());
    printf("  Motor Current: %d mA\n", front_loader_motor.get_current_draw());
    printf("  Motor Velocity: %.1f RPM\n", front_loader_motor.get_actual_velocity());
    printf("================================\n\n");
}

void Intake::resetEncoder() {
    front_loader_motor.tare_position();
    sensor_zero_value = front_loader_sensor.get_value();
    printf("Front Loader: Motor encoder reset and sensor recalibrated to %.0f\n", sensor_zero_value);
}

void Intake::calibrateSensorZero() {
    sensor_zero_value = front_loader_sensor.get_value();
    printf("Front Loader: Sensor zero position calibrated to %.0f\n", sensor_zero_value);
}

void Intake::calibratePosition(double current_position) {
    printf("Front Loader: Calibrating - setting current position as %.1f degrees\n", current_position);
    
    // Calculate what the motor position should be for this loader position
    double expected_motor_degrees = loaderDegreesToMotorDegrees(current_position);
    
    // Get current motor position
    double current_motor_degrees = getMotorPosition();
    
    printf("  Current motor encoder: %.1f degrees\n", current_motor_degrees);
    printf("  Expected motor position: %.1f degrees\n", expected_motor_degrees);
    printf("  Offset needed: %.1f degrees\n", expected_motor_degrees - current_motor_degrees);
    
    // Note: This is informational - you may need to manually adjust the FRONT_LOADER_GEAR_RATIO
    // or add an offset constant if the encoder doesn't align with your expected zero position
}

void Intake::setPosition(double target_degrees) {
    front_loader_target_position = target_degrees;
    
    // Safety check - limit target position to reasonable range
    if (target_degrees < -180 || target_degrees > 180) {
        printf("ERROR: Target position %.1f° is out of safe range (-180° to 180°)\n", target_degrees);
        return;
    }
    
    // Convert loader degrees to motor degrees
    double motor_target_degrees = loaderDegreesToMotorDegrees(target_degrees);
    
    printf("Front Loader: Setting position:\n");
    printf("  Target loader degrees: %.1f°\n", target_degrees);
    printf("  Target motor degrees: %.1f°\n", motor_target_degrees);
    printf("  Current motor position: %.1f°\n", getMotorPosition());
    printf("  Motor will move: %.1f° (%.2f rotations)\n", 
           motor_target_degrees - getMotorPosition(), 
           (motor_target_degrees - getMotorPosition()) / 360.0);
    
    // Safety check for excessive movement
    double movement = fabs(motor_target_degrees - getMotorPosition());
    if (movement > 7200) {  // More than 20 full rotations seems excessive
        printf("ERROR: Excessive movement detected (%.1f°). Check encoder!\n", movement);
        printf("  This might indicate encoder disconnection or invalid reading\n");
        return;
    }
    
    // Move motor to target position
    front_loader_motor.move_absolute(motor_target_degrees, FRONT_LOADER_MOTOR_SPEED);
    
    printf("  Motor command sent: move_absolute(%.1f, %d)\n", 
           motor_target_degrees, FRONT_LOADER_MOTOR_SPEED);
}

double Intake::loaderDegreesToMotorDegrees(double loader_degrees) const {
    // Motor rotates 12 times for every 1 rotation of the loader
    return loader_degrees * FRONT_LOADER_GEAR_RATIO;
}

double Intake::motorDegreesToLoaderDegrees(double motor_degrees) const {
    // Loader rotates 1/12 times for every motor rotation
    return motor_degrees / FRONT_LOADER_GEAR_RATIO;
}

void Intake::adjustPosition(double degrees) {
    // Get current target position and adjust it
    double new_target = front_loader_target_position + degrees;
    
    // Safety check - limit to reasonable range
    if (new_target < -180 || new_target > 180) {
        printf("Front Loader: Adjustment blocked - new position %.1f° would be out of safe range (-180° to 180°)\n", new_target);
        return;
    }
    
    // Apply the adjustment
    setPosition(new_target);
    
    printf("Front Loader: Position adjusted by %.1f° (from %.1f° to %.1f°)\n", 
           degrees, front_loader_target_position - degrees, front_loader_target_position);
}

void Intake::resetToOriginal() {
    // Toggle between the two preset positions (like the original toggle behavior)
    // This restores the original functionality while maintaining adjustability with L1/L2
    if (front_loader_deployed == FRONT_LOADER_DEPLOYED) {
        retract();  // Switch to retracted state
    } else {
        deploy();   // Switch to deployed state
    }
    
    printf("Front Loader: Reset to original position - %s\n", getCurrentStateString());
}