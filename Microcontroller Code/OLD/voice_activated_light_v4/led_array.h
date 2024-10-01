#pragma once

#include <stdint.h>
#include <Arduino.h>

class led_array {
public:
    uint32_t start_time;
    bool led_active;  // Should the LED be displayed or not? 
    
    uint32_t twinkle_timer;
    bool twinkle_flag;  
    
    // This is the constructor
    led_array() {
      start_time = millis();
      led_active = false;
      twinkle_flag = false;
    }    
};
