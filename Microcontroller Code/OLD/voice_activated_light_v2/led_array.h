#pragma once

#include <stdint.h>
#include <Arduino.h>

class led_array {
public:
    uint32_t start_time;
    bool led_active;  // Should the LED be displayed or not? 
    bool led_switch_off;
    
    // This is the constructor
    led_array() {
      start_time = millis();
      led_active = false;
      led_switch_off = true;
    }    
};
