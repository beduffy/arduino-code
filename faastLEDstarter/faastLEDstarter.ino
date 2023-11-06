#include <FastLED.h>
#define NUM_LEDS 60

CRGB leds[NUM_LEDS];

void setup() { 
  FastLED.addLeds<WS2812B, 6, GRB>(leds, NUM_LEDS); 
  FastLED.setBrightness(50);
}


void loop() {
	// leds[0] = CRGB::White; FastLED.show(); delay(30);
	// leds[0] = CRGB::Black; FastLED.show(); delay(30);
  // leds[0] = CRGB::Red;
  // leds[1] = CRGB::Blue;
  // FastLED.show();

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Red;
    if (i + 1 < NUM_LEDS) {
      leds[i + 1] = CRGB::Green;
    }
    FastLED.show();
    delay(50);
    leds[i] = CRGB::Black;
    if (i + 1 < NUM_LEDS) {
      leds[i + 1] = CRGB::Black;
    }
  }

  for (int i = NUM_LEDS - 1; i > 0; i--) {
    leds[i] = CRGB::Red;
    if (i + 1 < NUM_LEDS) {
      leds[i + 1] = CRGB::Green;
    }
    FastLED.show();
    delay(50);
    leds[i] = CRGB::Black;
    if (i + 1 < NUM_LEDS) {
      leds[i + 1] = CRGB::Black;
    }
  }
}