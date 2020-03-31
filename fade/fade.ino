
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
 
#define PIN 6
#define NUMPIXELS 24
 
 
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
 
#define DELAYVAL 1 // Time (in milliseconds) to pause between pixels
 
void NeoFade(int FadeSpeed)
{
  int fspeed;
  for (int i = 0; i < NUMPIXELS; i++) { pixels.setPixelColor(i, 255, 255, 255); } for (int j = 255; j > 0; j=j-2)
  {
    pixels.setBrightness(j);
    pixels.show();
    delay(FadeSpeed);
  }
}
 
void setup() {
  pixels.begin();
}
 
void loop() {
  NeoFade(10);
}
