#pragma once

#include <Arduino.h>

#define DEBUG                       false // Control if we see debug messages or not
#define DEBUG_SOUND                 false // Control if we see debug messages or not
#define DEBUG_LED                   true // Control if we see debug messages or not

// Below is for testing:
#define DEBUG_TESTING   false

#define HEARTBEAT_LED_PIN           LED_BUILTIN

#define SOUND_PIN       27    // Has the sound sensor on this pin
#define POT_PIN         26    // Has the sensitivity potentiometer on this pin

// Below is for testing:
#define DEBUG_TESTING   false

//// Below is for PiMoRoNi board (testing)
//#define SOUND_PIN       27    // Has the sound sensor on this pin
//#define POT_PIN         26    // Has the sensitivity potentiometer on this pin

#define MIN_SOUND       50    // Removes lower sounds

// Which pin on the Arduino is connected to the NeoPixels?
#define LED_PIN         15
#define LED_1_PIN       5
#define LED_2_PIN       6
#define LED_3_PIN       7

// How many NeoPixels are attached?
#define NUM_LEDS        20          // NeoPixels PER STRAND, total number is 8X this!
#define COLOR_ORDER     NEO_GRB // NeoPixel color format (see Adafruit_NeoPixel)

#define NUM_READINGS    50       //Number of readings to store in background buffer

#define FADE_IN_TIME    1000    // mS to fade in the light
#define TWINKLE_TIME    5000    // mS to keep the twinkle running, unless retriggered
//#define FADE_IN_TIME    100    // mS to fade in the light TESTING
//#define TWINKLE_TIME    200    // mS to keep the twinkle running, unless retriggered TESTING


#define TWINKLE_DELAY_MAX   500     // max time in mS
#define TWINKLE_DELAY_MIN   50     // max time in mS
#define TWINKLE_SPARK_MAX   250      // max time in mS  
#define TWINKLE_SPARK_MIN   50      // max time in mS  
