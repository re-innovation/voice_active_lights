/*
  Voice Activated Light - for Makers of Imaginary Worlds

  This code is for the Adafruit Feather RP2040 SCORPIO - 8 Channel NeoPixel Driver board.

  Device 1:
  Sound sensor - to control LED colours when sound is heard.
  LEDs are on three strands each with 20 LEDs.
  Sound volume will set how many lights turn on. Lights will turn on a twinkle for 5 seconds then fade out unless volume has increased.
  Each strand has two different colours to twinkle between.

  Sensor used:
  https://wiki.seeedstudio.com/Grove-Loudness_Sensor/
  Uses connector on pin A0.

  Potentiometer on pin A1 - Sets sensitivity of maximum

  LED x 3 are on LED outputs 0/1/2
  Testing - LEDs are on pin GP15

*/
#include <Arduino.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
#include <stdint.h>

#include "Config.h"

// ******** This is for Scheduling Tasks **************************
// Must include this library from Arduino IDE Library Manager
// https://github.com/arkhipenko/TaskScheduler
#include <TaskScheduler.h>
// Callback methods prototypes
void t1Callback();
void t1SCallback();
Task t1     (5 * TASK_MILLISECOND, TASK_FOREVER,  &t1Callback);         // Sample as base rate of 10Hz
Task t1S    (200 * TASK_MILLISECOND, TASK_FOREVER, &t1SCallback);         // Send data every 500mS to display
Scheduler runner;

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

uint32_t  red = strip.Color(255, 0, 0); //red
uint32_t  orange = strip.Color(255, 135, 0); //orange
uint32_t  yellow = strip.Color(255, 255, 0); //yellow
uint32_t  green = strip.Color(0, 255, 0);//green
uint32_t  blue = strip.Color(0, 0, 255); //blue
uint32_t  cyan = strip.Color(0, 255, 255); //cyan
uint32_t  purple = strip.Color(128, 0, 128); //purple
//red->orange->yellow-> green-> blue->indigo->violet
uint32_t* RainbowColours[] = {&red, &orange, &yellow, &green, &blue, &cyan, &purple};


// *********** This is for the data holders *************************
volatile float sound_background[NUM_READINGS];    // This creates our background sound array (for averaging)
volatile float sound_sample;
volatile uint32_t sound_sample_holder;
volatile int sound_max;

volatile int data_counter_1s = 0;

int sound_sensitivity;

volatile bool led_trigger_flag = false;
volatile bool on_flag = false;
volatile int led_speed = 10;    // Speed for doing wipe - slow if quite, quick if loud

int leds_switched_on;    // This holds how many LEDs should be switched on with the current sound level.

void t1Callback()
{
  sound_sample_holder += analogRead(SOUND_PIN);
  data_counter_1s++;
}

void t1SCallback() {
  sound_sensitivity = analogRead(POT_PIN);
  sound_sensitivity = constrain(sound_sensitivity, 0, 4095);

  if (DEBUG == true)
  {
    Serial.print("SENSE:");
    Serial.println(sound_sensitivity);
  }

  if (!t1S.isFirstIteration())
  {
    sound_sample = (float)sound_sample_holder / (float)data_counter_1s;

    // Sort out the background sound array - shift it all along
    for (int y = 1; y <= NUM_READINGS; y++)
    {
      sound_background[NUM_READINGS - y ] = sound_background[NUM_READINGS - y - 1];
    }
    sound_background[0] = sound_sample;

    // Calculate the average background sound for past NUM_READINGS
    float sound_average = 0.0;
    for (int y = 0; y < NUM_READINGS; y++)
    {
      sound_average += sound_background[y];
    }
    sound_average = sound_average / (float)NUM_READINGS;

    if (DEBUG == true)
    {
      // Show the sound_background array
      for (int y = 0; y < NUM_READINGS; y++)
      {
        Serial.print(sound_background[y]);
        Serial.print(":");
      }
      Serial.print("N:\t");
      Serial.println(data_counter_1s);
      Serial.print("AVE:\t");
      Serial.print(sound_sample);
      Serial.print("\t BAK:\t");
      Serial.println(sound_average);
    }

    // Do LEDs need to be switched on?
    int sound_above = (sound_sample - sound_average);
    // here we can include mapping for the sensitivity
    // leds_switched_on  = map(sound_above, 0, sound_sensitivity, SOUND_MIN, SOUND_MAX); // FUTURE TESTING
    leds_switched_on  = map(sound_above, 0, 4095, SOUND_MIN, SOUND_MAX);
    leds_switched_on = constrain(leds_switched_on, SOUND_MIN, SOUND_MAX);
    
    if (DEBUG == true)
    {
      Serial.print("LED:\t");
      Serial.println(leds_switched_on);
    }

    // Reset everything for next sample period
    sound_max = 0;
    sound_sample_holder = 0;
    data_counter_1s = 0;   // Reset the counter
    digitalWrite(HEARTBEAT_LED_PIN, !digitalRead(HEARTBEAT_LED_PIN));
  }
}

void setup()
{
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(255); // Set BRIGHTNESS to about 1/5 (max = 255)

  pinMode(HEARTBEAT_LED_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_LED_PIN, LOW);
  Serial.begin(115200);
  delay(200);

  // Use external reference voltage for analog reads
  //analogReference(INTERNAL2V5);
  //analogReference(INTERNAL4V3);
  analogReadResolution(12); // Gives better resolution (default is 10).

  // Set up Scheduler:
  runner.init();
  Serial.print("Init scheduler->");
  runner.addTask(t1);
  runner.addTask(t1S);
  Serial.print("Added Tasks->");
  t1.enable();
  t1S.enable();
  Serial.println("Enabled Tasks");
  // Reset the sound_background array:
  for (int y = 0; y < NUM_READINGS; y++)
  {
    sound_background[y] = 0;
  }
}

void loop()
{
  // Deal with scheduled tasks
  runner.execute();

}
