
#ifdef M5UNIFIED
#include <M5Unified.h>
#endif
#include <Arduino.h>

#include "freertos-all.h"

#define PSRAM_AVAILABLE(size) (size < ESP.getMaxAllocPsram())

Queue queue(100, sizeof(size_t), true);
Task task("name", 1000, 10, []() {
  static size_t count = 0;
  delay(500);

  queue.Enqueue(&(++count));
});

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
  Serial.printf("total PSram: %d  free: %d \n", ESP.getPsramSize(),
                ESP.getFreePsram());
  Queue queue2(100, sizeof(size_t), true);
  Serial.printf("total PSram: %d  free: %d \n", ESP.getPsramSize(),
                ESP.getFreePsram());
  task.Start(0);
  delay(1000);
}

void loop() {
  size_t result;
  queue.Dequeue(&result);
  Serial.println(result);
  delay(1000);
}
