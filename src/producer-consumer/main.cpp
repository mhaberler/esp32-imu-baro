
#ifdef M5UNIFIED
#include <M5Unified.h>
#endif
#include "Ticker.h"

#define PRODUCER_CORE 0
#define PRODUCER_PRIORITY 1
#define PRODUCER_STACKSIZE 8192
#define PRODUCER_TASKNAME "producer"
#define PRODUCER_TICK 1 // mS

#define CONSUMER_CORE 0
#define CONSUMER_PRIORITY 1
#define CONSUMER_STACKSIZE 8192
#define CONSUMER_TASKNAME "consumer"
#define CONSUMER_TICK 200 // mS

Ticker producer_ticker, consumer_ticker;
bool run_consumer, run_producer;
bool initProducerTask(void);
bool initConsummerTask(void);

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
  bool success = initProducerTask();
  Serial.printf("producer task started: %s\n", success ? "OK" : "failed");
  success = initConsummerTask();
  Serial.printf("consumer task started: %s\n", success ? "OK" : "failed");

  consumer_ticker.attach_ms(CONSUMER_TICK, []() { run_consumer = true; });
  producer_ticker.attach_ms(PRODUCER_TICK, []() { run_producer = true; });
}

void loop() { yield(); }

void producer(void *p) {
  while (true) {
    if (run_producer) {
      // do produce
      run_producer = false;
    }
    yield();
  }
}
void consumer(void *p) {
  while (true) {
    if (run_consumer) {
      // do consume
      run_consumer = false;
    }
    yield();
  }
}

TaskHandle_t pTaskHandle = NULL;

bool initProducerTask(void) {
  xTaskCreateUniversal(producer, PRODUCER_TASKNAME, PRODUCER_STACKSIZE, NULL,
                       PRODUCER_PRIORITY, &pTaskHandle, PRODUCER_CORE);
  return pTaskHandle != NULL;
}

TaskHandle_t cTaskHandle = NULL;

bool initConsummerTask(void) {
  xTaskCreateUniversal(consumer, CONSUMER_TASKNAME, CONSUMER_STACKSIZE, NULL,
                       CONSUMER_PRIORITY, &cTaskHandle, CONSUMER_CORE);
  return cTaskHandle != NULL;
}
