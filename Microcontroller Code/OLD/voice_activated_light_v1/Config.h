#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define DEBUG                       true // Control if we see debug messages or not
#define HEARTBEAT_LED_PIN           LED_BUILTIN

#define SOUND_PIN     26    // Has the sound sensor on this pin
#define POT_PIN       27    // Has the sensitivity potentiometer on this pin

// Which pin on the Arduino is connected to the NeoPixels?
#define LED_PIN      15
#define LED_1_PIN    5
#define LED_2_PIN    6
#define LED_3_PIN    7

// How many NeoPixels are attached?
#define LED_COUNT   50

#define NUM_READINGS                50       //Number of readings to store in background buffer

#define SOUND_MIN                   0
#define SOUND_MAX                   20       // Number of levels to light

#define LED_SPEED_KNEE              100
#define LED_SPEED_MIN_1             200     // mS for quickest time - quiet sounds
#define LED_SPEED_MAX_1             800     // mS for slowest time - quiet sounds
#define LED_SPEED_MIN_2             10     // mS for quickest time - Loud sounds
#define LED_SPEED_MAX_2             100     // mS for slowest time - Loud sounds
