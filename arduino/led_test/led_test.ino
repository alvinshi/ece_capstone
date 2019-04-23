#include <FastLED.h>

// LED Setup
#define LED_PIN_1   4
#define LED_PIN_2   5
#define COLOR_ORDER RGB
#define CHIPSET     WS2811
#define NUM_LEDS    3

#define BRIGHTNESS  200
#define FRAMES_PER_SECOND 60

CRGB leds_1[NUM_LEDS];
CRGB leds_2[NUM_LEDS];

void setup() {
  delay(2000);
  FastLED.addLeds<CHIPSET, LED_PIN_1, COLOR_ORDER>(leds_1, NUM_LEDS);
  FastLED.addLeds<CHIPSET, LED_PIN_2, COLOR_ORDER>(leds_2, NUM_LEDS);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i < NUM_LEDS; i++) {
        leds_1[i] = CRGB(250,60,0);
        leds_2[i] = CRGB(250,60,0);
        //leds_1[i] = CRGB::White;
        //leds_2[i] = CRGB::White;
  }
  FastLED.show();
  delay(100);
}
