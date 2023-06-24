
#ifdef M5UNIFIED
#include <M5Unified.h>
#else
#include <Arduino.h>
#endif

void setup() {
#ifdef M5UNIFIED
  auto cfg = M5.config();
  M5.begin(cfg);
#else
  Serial.begin(115200);
#endif
  while (!Serial) {
    yield();
  }
  Serial.printf ("Hello, World!\n");
}

void loop() {
  delay (1000);
  Serial.printf("Hello, World loop!\n");
 }
