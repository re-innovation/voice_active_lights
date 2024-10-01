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

// Uses Adafruit_NeoPXL8 which must be installed
#include <Adafruit_NeoPXL8.h>

#include <stdint.h>

#include "Config.h"
#include "led_array.h"

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

// ******** This is for Neopixel LEDs **************************
//int8_t pins[8] = { LED_PIN, -1, -1, -1, -1, -1, -1, -1 };
// For the Feather RP2040 SCORPIO, use this list:
int8_t pins[8] = { 16, 17, 18, 19, 20, 21, 22, 23 };

Adafruit_NeoPXL8 leds(NUM_LEDS, pins, COLOR_ORDER);

static uint8_t colors[6][3] = {
  255, 30, 0,     // Row 0: Strand 1 colour 1: Orange
  0,   0, 255,    // Row 1: Strand 1 colour 2: Blue
  255, 0,   0,    // Row 2: Strand 2 colour 1: Red
  0, 255,   0,    // Row 3: Strand 2 colour 2: Green
  192,   0, 255,  // Row 4: Strand 3 colour 1: Purple
  255, 255, 0,    // Row 5: Strand 3 colour 2: Yellow
};

//// TESTING ALL WHITE
//static uint8_t colors[6][3] = {
//  255, 255, 255,     // Row 0: Strand 1 colour 1: Orange
//  255, 255, 255,    // Row 1: Strand 1 colour 2: Blue
//  255, 255, 255,     // Row 2: Strand 2 colour 1: Red
//  255, 255, 255,    // Row 3: Strand 2 colour 2: Green
//  255, 255, 255,    // Row 4: Strand 3 colour 1: Purple
//  255, 255, 255,    // Row 5: Strand 3 colour 2: Yellow
//};


// *********** This is for the data holders *************************
volatile float sound_background[NUM_READINGS];    // This creates our background sound array (for averaging)
volatile float sound_sample;
volatile uint32_t sound_sample_holder;
volatile int sound_max;
volatile int data_counter_1s = 0;

volatile int sound_sensitivity;
volatile int leds_switched_on = 0;    // This holds how many LEDs should be switched on with the current sound level.
int old_leds_switched_on = 0;
led_array my_led_array[NUM_LEDS];        // This array holds the state of each LED (off, fade, twinkle)
uint32_t  old_millis = 0;

void t1Callback()
{
  if (DEBUG_TESTING == false)
  {
    sound_sample_holder += analogRead(SOUND_PIN);
  }
  else
  {
    sound_sample_holder += analogRead(POT_PIN);
  }
  data_counter_1s++;
}

void t1SCallback()
{
  if (DEBUG_TESTING == false)
  {
    sound_sensitivity = analogRead(POT_PIN);
    sound_sensitivity = constrain(sound_sensitivity, MIN_SOUND, 4095);
  }
  else
  {
    sound_sensitivity = 4095;
  }

  if (DEBUG_SOUND == true)
  {
    Serial.print("SENSE:");
    Serial.println(sound_sensitivity);
  }

  if (!t1S.isFirstIteration())
  {
    sound_sample = (float)sound_sample_holder / (float)data_counter_1s;

    // Sort out the background sound array - shift it all along
    for (int y = 0; y < NUM_READINGS; y++)
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


    // Do LEDs need to be switched on?
    int sound_above = (sound_sample - sound_average);
    //int sound_above = (sound_sample); // Try doing this from zero (no background filter)

    // here we can include mapping for the sensitivity
    // leds_switched_on  = map(sound_above, 0, sound_sensitivity, SOUND_MIN, SOUND_MAX); // FUTURE TESTING
    leds_switched_on  = map(sound_above, MIN_SOUND, sound_sensitivity, 0, NUM_LEDS);
    leds_switched_on = constrain(leds_switched_on, 0, NUM_LEDS);

    if (DEBUG_SOUND == true)
    {
      //      // Show the sound_background array
      //      for (int y = 0; y < NUM_READINGS; y++)
      //      {
      //        Serial.print(sound_background[y]);
      //        Serial.print(":");
      //      }
      //      Serial.print("N:\t");
      //      Serial.println(data_counter_1s);
      Serial.print("DATA:\t");
      Serial.print(sound_sample);
      Serial.print("\t BAK:\t");
      Serial.print(sound_average);
      Serial.print("\t ABOVE:\t");
      Serial.print(sound_above);
      Serial.print("\t LED:\t");
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
  // Start NeoPXL8. If begin() returns false, either an invalid pin list
  // was provided, or requested too many pixels for available RAM.
  if (!leds.begin()) {
    // Blink the onboard LED if that happens.
    pinMode(LED_BUILTIN, OUTPUT);
    for (;;) digitalWrite(LED_BUILTIN, (millis() / 500) & 1);
  }
  // Otherwise, NeoPXL8 is now running, we can continue.
  leds.setBrightness(255);  // Set brightness to MAX

  // Blank all the LEDs
  for (int i = 0; i < 8; i++) {
    if (pins && (pins[i] < 0)) continue; // No pixels on this pin
    leds.fill(0);
    leds.show();
  }

  pinMode(HEARTBEAT_LED_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_LED_PIN, LOW);
  Serial.begin(115200);
  delay(500);

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
  rp2040.wdt_begin(3000); // Watchdog timer - 3s
}

void loop()
{
  // Feed the WDT
  rp2040.wdt_reset();

  // Deal with scheduled tasks- sound sensor
  runner.execute();

  // Deal with LEDs
  update_leds();
}


void update_leds()
{
  uint32_t now = millis(); // Get time once at start of each frame

  uint32_t recent_start_time = 0;
  for (int i = 0; i < NUM_LEDS; i++)
  {
    // Check for most recent start_time
    if (my_led_array[i].start_time > recent_start_time)
    {
      recent_start_time = my_led_array[i].start_time;
    }
  }
  
  for (int p = 0; p < NUM_LEDS; p++)
  {
    uint32_t led_time = now - my_led_array[p].start_time; // Calculate difference between now and start time

    // Check if LED should be on
    if (p < leds_switched_on)
    {
      if (my_led_array[p].led_active == false)
      {
        // If LED was not on then should set it on now with now as start time
        my_led_array[p].start_time = now;
        my_led_array[p].led_active = true;
      }
      else
      {
        my_led_array[p].led_active = true;
      }
    }

    if (my_led_array[p].led_active == true)
    {
      // if LED was on then it needs to be on still: check start time to see where it is
      if (led_time < FADE_IN_TIME)
      {
        // In this case we are still in fade in routine - leave it to fade in
      }
      else if (led_time >= FADE_IN_TIME && led_time < (FADE_IN_TIME + TWINKLE_TIME))
      {
        // If we are in twinkle time and there has been a more recent activation then update the timer:
        if (p < leds_switched_on && my_led_array[p].start_time < recent_start_time)
        {
          my_led_array[p].start_time = recent_start_time;  // Puts it into twinkling time slot (start of it)
        }
      }
      else if (led_time >= (FADE_IN_TIME + TWINKLE_TIME) && led_time < (FADE_IN_TIME + TWINKLE_TIME + FADE_IN_TIME))
      {
        if (p < leds_switched_on)
        {
          // If this is the case then we are in fade out  - need to retrigger and fade back in:
          my_led_array[p].start_time = now + FADE_IN_TIME + TWINKLE_TIME - led_time;
        }
      }
    }


    // Sort the LED effects:

    // Update the LEDs depending upon the array:
    // For each pixel of row...
    if (my_led_array[p].led_active == true && (led_time < FADE_IN_TIME))
    {
      // fade up the LED - all 3 strands
      // Want brightness to go from 0 -> 255 in time from 0 -> FADE_IN_TIME
      float brightness = ((float)(now - my_led_array[p].start_time) / (float)FADE_IN_TIME) * 255.0;
      leds.setPixelColor(p, leds.Color(((brightness * colors[0][0]) / 255), ((brightness * colors[0][1]) / 255), ((brightness * colors[0][2]) / 255)));
      leds.setPixelColor(NUM_LEDS + p, leds.Color(((brightness * colors[2][0]) / 255), ((brightness * colors[2][1]) / 255), ((brightness * colors[2][2]) / 255)));
      leds.setPixelColor(2 * NUM_LEDS + p, leds.Color(((brightness * colors[4][0]) / 255), ((brightness * colors[4][1]) / 255), ((brightness * colors[4][2]) / 255)));
      my_led_array[p].twinkle_timer_1 = millis() + random(TWINKLE_DELAY_MIN, TWINKLE_DELAY_MAX);
    }
    else if (my_led_array[p].led_active == true && (led_time < (FADE_IN_TIME + TWINKLE_TIME)))
    {
      // Twinkle the LEDs
      // Want a random delay then randon time for sparkle (up to a point) and random colour.
      if (millis() > my_led_array[p].twinkle_timer_1 && my_led_array[p].twinkle_flag_1 == false)
      {
        leds.setPixelColor(p, leds.Color(colors[1][0], colors[1][1], colors[1][2]));
        //        leds.setPixelColor(NUM_LEDS + p, leds.Color(colors[3][0], colors[3][1], colors[3][2]));
        //        leds.setPixelColor(2 * NUM_LEDS + p, leds.Color(colors[5][0], colors[5][1], colors[5][2]));
        my_led_array[p].twinkle_flag_1 = true;
        my_led_array[p].twinkle_timer_1 = millis() + random(TWINKLE_SPARK_MIN, TWINKLE_SPARK_MAX);
      }
      if (millis() > my_led_array[p].twinkle_timer_1 && my_led_array[p].twinkle_flag_1 == true)
      {
        leds.setPixelColor(p, leds.Color(colors[0][0], colors[0][1], colors[0][2]));
        //        leds.setPixelColor(NUM_LEDS + p, leds.Color(colors[2][0], colors[2][1], colors[2][2]));
        //        leds.setPixelColor(2 * NUM_LEDS + p, leds.Color(colors[4][0], colors[4][1], colors[4][2]));
        my_led_array[p].twinkle_flag_1 = false;
        my_led_array[p].twinkle_timer_1 = millis() + random(TWINKLE_DELAY_MIN, TWINKLE_DELAY_MAX);
      }
    }
    else if (my_led_array[p].led_active == true && (led_time < (FADE_IN_TIME + TWINKLE_TIME + FADE_IN_TIME)))
    {
      // fade down the LED - all 3 strands
      // Want brightness to go from 255 -> 0 in time from FADE_IN_TIME + TWINKLE_TIME -> FADE_IN_TIME + TWINKLE_TIME + FADE_IN_TIME
      float brightness = ((float)(led_time - FADE_IN_TIME - TWINKLE_TIME) / (float)FADE_IN_TIME) * 255.0;
      brightness = 255 - brightness;  // Invert it...
      leds.setPixelColor(p, leds.Color(((brightness * colors[0][0]) / 255), ((brightness * colors[0][1]) / 255), ((brightness * colors[0][2]) / 255)));
      leds.setPixelColor(NUM_LEDS + p, leds.Color(((brightness * colors[2][0]) / 255), ((brightness * colors[2][1]) / 255), ((brightness * colors[2][2]) / 255)));
      leds.setPixelColor(2 * NUM_LEDS + p, leds.Color(((brightness * colors[4][0]) / 255), ((brightness * colors[4][1]) / 255), ((brightness * colors[4][2]) / 255)));
    }
    else if (my_led_array[p].led_active == true && (led_time > (FADE_IN_TIME + TWINKLE_TIME + FADE_IN_TIME)))
    {
      if (p >= leds_switched_on)
      {
        // Switch off the LEDs and stop them being active
        my_led_array[p].led_active = false;
        //my_led_array[p].start_time = 0;
        leds.setPixelColor(p, leds.Color(0, 0, 0));
        leds.setPixelColor(NUM_LEDS + p, leds.Color(0, 0, 0));
        leds.setPixelColor(2 * NUM_LEDS + p, leds.Color(0, 0, 0));
      }
    }
  }
  leds.show();

  if (DEBUG_LED == true)
  {
    if (millis() > old_millis + 500)
    {
      //      Serial.print("Time Now:");
      //      Serial.println(now);
      for (int y = 0; y < NUM_LEDS; y++)
      {
        Serial.print(my_led_array[y].start_time);
        Serial.print("\t :");
      }
      Serial.println();
      for (int y = 0; y < NUM_LEDS; y++)
      {
        Serial.print(my_led_array[y].led_active);
        Serial.print("\t :");
      }
      Serial.println();
      Serial.print("Recent Start:");
      Serial.println(recent_start_time);
      old_millis = millis();
    }
  }
}
