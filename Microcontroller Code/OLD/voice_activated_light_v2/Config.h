#pragma once

#include <Arduino.h>

#define DEBUG                       true // Control if we see debug messages or not
#define HEARTBEAT_LED_PIN           LED_BUILTIN

#define SOUND_PIN       26    // Has the sound sensor on this pin
#define POT_PIN         27    // Has the sensitivity potentiometer on this pin

#define MIN_SOUND       50    // Removes lower sounds

// Which pin on the Arduino is connected to the NeoPixels?
#define LED_PIN         15
#define LED_1_PIN       5
#define LED_2_PIN       6
#define LED_3_PIN       7

// How many NeoPixels are attached?
#define NUM_LEDS        20          // NeoPixels PER STRAND, total number is 8X this!
#define COLOR_ORDER     NEO_RGB // NeoPixel color format (see Adafruit_NeoPixel)

#define NUM_READINGS    50       //Number of readings to store in background buffer

#define FADE_IN_TIME    1000    // mS to fade in the light
#define TWINKLE_TIME    5000    // mS to keep the twinkle running, unless retriggered
