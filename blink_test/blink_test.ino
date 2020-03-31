#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define PIN 6
Adafruit_NeoPixel pixels(24, PIN, NEO_GRB + NEO_KHZ800);

void NeoBlink(int num, int wait)
{
  for (int r = 0; r < num; r++)
  {
    pixels.setPixelColor(r, 255, 0, 0);
  }
  pixels.show();
  delay(wait);

  for (int Or = 0; Or < num; Or++)
  {
    pixels.setPixelColor(Or, 255, 69, 0);
  }
  pixels.show();
  delay(wait);

  for (int y = 0; y < num; y++){
    pixels.setPixelColor(y, 255, 255, 0);
  }
  pixels.show();
  delay(wait);

  for (int g = 0; g < num; g++){
    pixels.setPixelColor(g, 0, 255, 0);
  }
  pixels.show();
  delay(wait);


  for (int b = 0; b < num; b++){
    pixels.setPixelColor(b, 0, 0, 255);
  }
  pixels.show();
  delay(wait);


  for (int p = 0; p < num; p++){
    pixels.setPixelColor(p, 100, 0, 100);
  }
  pixels.show();
  delay(wait);

}


void setup()
{
  pixels.begin();
  pixels.setBrightness(50);
}

void loop()
{
  NeoBlink(24, 200);
}
