#include <Wire.h>
#include <VL53L1X.h>
#include "quaternion.h"

extern Quaternion attitude;

VL53L1X sensor;
float laser_alt_mm = 0.0f;
float laser_vel_mms = 0.0f; // Vertical speed in mm/s (positive = climbing, negative = dropping)
unsigned long last_laser_us = 0;

void setupLaser() {
    print("Setup VL53L1X Laser\n");
    
    sensor.setTimeout(500);
    
    if (!sensor.init()) {
        print("FAILED: VL53L1X not detected on I2C bus!\n");
    } else {
        sensor.setDistanceMode(VL53L1X::Long);
        sensor.setMeasurementTimingBudget(33000); // 33ms budget (~30 Hz)
        sensor.startContinuous(33);
        last_laser_us = micros();
        print("VL53L1X Laser connected and ranging.\n");
    }
}

void readLaser() {
    // Non-blocking check: Only read if a new 33ms measurement is ready
    if (sensor.dataReady()) {
        uint16_t current_distance = sensor.read(false); 
        
        if (!sensor.timeoutOccurred() && current_distance < 4000) {
            unsigned long now_us = micros();
            float laser_dt = (now_us - last_laser_us) * 1e-6f;
            last_laser_us = now_us;

            // 1. Apply physical calibration offset (-8 mm)
            float raw_mm = (float)current_distance - 8.0f;
            if (raw_mm < 0.0f) raw_mm = 0.0f;
            
            // 2. Tilt Compensation (Cosine projection of Roll and Pitch)
            float tilt_cos = cos(attitude.getRoll()) * cos(attitude.getPitch());
            // Clamp to 0.5f (60 deg) for safety in case of extreme angles
            if (tilt_cos < 0.5f) tilt_cos = 0.5f; 
            float corrected_distance = raw_mm * tilt_cos;
            
            // 3. Smooth altitude (70% old, 30% new)
            float prev_alt_mm = laser_alt_mm;
            laser_alt_mm = (laser_alt_mm * 0.70f) + (corrected_distance * 0.30f);

            // 4. Calculate and smooth Vertical Velocity (mm/s) for D-term braking
            if (laser_dt > 0.005f && laser_dt < 0.5f) {
                float raw_vel_mms = (laser_alt_mm - prev_alt_mm) / laser_dt;
                laser_vel_mms = (laser_vel_mms * 0.70f) + (raw_vel_mms * 0.30f);
            }
        }
    }
}