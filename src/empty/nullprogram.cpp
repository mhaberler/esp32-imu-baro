
#ifdef M5UNIFIED
#include <M5Unified.h>
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
}

void loop() { yield(); }
