#pragma once

#include <stdint.h>
#include <Arduino.h>

class led_array {
  public:
    uint32_t start_time;
    bool led_active;  // Should the LED be displayed or not?
    bool twinkle_flag;

    uint32_t twinkle_timer_1;
    bool twinkle_flag_1;
    uint32_t twinkle_timer_2;
    bool twinkle_flag_2;
    uint32_t twinkle_timer_3;
    bool twinkle_flag_3;


    // This is the constructor
    led_array() {
      start_time = millis();
      led_active = false;
      twinkle_flag = false;
      twinkle_flag_1 = false;
      twinkle_flag_2 = false;
      twinkle_flag_3 = false;     
    }
};
