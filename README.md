# Voice Activated LED lights
Colour changing voice activated lights for Makers of Imaginary Worlds

Sound sensor - to control LED colours when sound is heard.
 
LEDs are on three strands each with 20 LEDs.

Sound volume will set how many lights turn on. Lights will turn on, fade in, twinkle between 2 colours for 5 seconds then fade out unless volume has increased.

Fade in/out time is the same and can be edited in code. Twinkle time can be edited in code.

Each strand has two different colours to twinkle between.

Sound sensor records preious 30 seconds to give background level. Sound must be higher than this background level to activate the lights. This dymanically removes background environmental noise. But it does mean that voice into the unit will gradually have less effect. If the sound goes quiet for a while then it can re-trigger.

# Microcontroller

Adafruit Feather RP2040 SCORPIO - 8 Channel NeoPixel Driver

https://www.adafruit.com/product/5650

Need to include this board using board manager.
Follow these instructions:
https://learn.adafruit.com/introducing-feather-rp2040-scorpio/overview

Also need to inlcude some libraries - all via library manager.
    
# Sound Sensor

https://wiki.seeedstudio.com/Grove-Loudness_Sensor/

Uses connector on pin A1/pin 27.
  
Potentiometer on pin A0/pin 26 - Sets sensitivity of maximum

# LEDs

LED x 3 are on LED outputs 0/1/2 

LEDs are 1m long with 20 LEDs.

Using these LEDs:

https://thepihut.com/products/adafruit-neopixel-slim-led-dot-strand-20-leds-at-2-pitch-1m
