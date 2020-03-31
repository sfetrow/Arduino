
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
 
#define PIN 6
#define NUMPIXELS 24
 
 
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
 
#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels
 
void setup() {
  pixels.begin();
}
 
void loop() {
  pixels.clear();
  pixels.setPixelColor(random(0, NUMPIXELS), random(0, 255), random(0, 255), random(0, 255));
  pixels.show();
  delay(10);
}
