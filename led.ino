#include <WiFi.h>

#define EXTERNAL_LED_PIN 1
#define BUILTIN_LED_PIN 8 

bool connected = false; 

void setupLED() {
    pinMode(EXTERNAL_LED_PIN, OUTPUT);
    pinMode(BUILTIN_LED_PIN, OUTPUT);
    
    // External LED is Active-HIGH (LOW = OFF)
    digitalWrite(EXTERNAL_LED_PIN, LOW); 
    
    // ESP32-C3 Built-in LED is Active-LOW (HIGH = OFF)
    digitalWrite(BUILTIN_LED_PIN, HIGH); 
}

void loopLED() {
    unsigned long currentMillis = millis();
    static unsigned long armedStartTime = 0;
    static bool wasArmed = false;
    
    int targetBrightness = 0; 

    // Detect the exact millisecond the motors are armed
    if (armed && !wasArmed) {
        armedStartTime = currentMillis;
    }
    wasArmed = armed;

    // --- STATE 1: ARMED (Motors Live) ---
    if (armed) {
        unsigned long timeSinceArmed = currentMillis - armedStartTime;
        
        if (timeSinceArmed < 600) {
            if (timeSinceArmed < 100 || (timeSinceArmed > 200 && timeSinceArmed < 300)) {
                targetBrightness = 255;
            } else {
                targetBrightness = 0;
            }
        } else {
            if ((currentMillis / 80) % 2 == 0) {
                targetBrightness = 255;
            } else {
                targetBrightness = 0;
            }
        }
    }
    
    // --- STATE 2: CONNECTED (Wi-Fi Linked or QGC connected) ---
    // Detects if drone joined a router OR if a phone joined the drone's hotspot
    else if (WiFi.status() == WL_CONNECTED || WiFi.softAPgetStationNum() > 0 || connected) {
        targetBrightness = 255; // Solid light
    }
    
    // --- STATE 3: IDLE / WAITING (No connection) ---
    else {
        // Smooth 2-second breathing fade
        unsigned long cycle = currentMillis % 2000;
        
        if (cycle < 1000) {
            targetBrightness = map(cycle, 0, 1000, 0, 255); 
        } else {
            targetBrightness = map(cycle, 1000, 2000, 255, 0); 
        }
    }

    // Apply normal brightness to the external LED
    analogWrite(EXTERNAL_LED_PIN, targetBrightness);
    
    // INVERT the brightness (255 - target) for the backwards built-in LED
    analogWrite(BUILTIN_LED_PIN, 255 - targetBrightness);
}