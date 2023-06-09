
#include "defs.hpp"
#ifdef FASTLED_TYPE

#include <FastLED.h>

// Define the array of leds
CRGB leds[FASTLED_NUM_LEDS];
int num_leds = -1;
#endif

void fastled_setup(void) {
#ifdef FASTLED_TYPE
  num_leds = FASTLED_NUM_LEDS;
#ifdef FASTLED_CLOCK_PIN
  FastLED.addLeds<FASTLED_TYPE, FASTLED_DATA_PIN, FASTLED_CLOCK_PIN, RGB>(
      leds, FASTLED_NUM_LEDS); // GRB ordering is typical
#else
  FastLED.addLeds<FASTLED_TYPE, FASTLED_DATA_PIN>(
      leds, RGB > (leds, FASTLED_NUM_LEDS));
#endif
#endif
}

void fastled_set(activitySource_t source, bool active) {
#ifdef FASTLED_TYPE
  if (num_leds > ACT_ENDMARKER) {
    // can use individual leds
    leds[source] = CRGB::Red;
  }
  if (num_leds == 1) {
    // need to mash it into a single led color
  }
#endif
}

void fastled_update(void) {
#ifdef FASTLED_TYPE
  FastLED.show();
#endif
}
