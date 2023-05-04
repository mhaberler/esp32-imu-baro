
#ifdef M5UNIFIED
#include <M5Unified.h>
#endif
#include <lockless_tripplebuffer/TripleBuffer.h>

#define PRODUCER_CORE 0
#define PRODUCER_PRIORITY 1
#define PRODUCER_STACKSIZE 8192
#define PRODUCER_TASKNAME "producer"
#define PRODUCER_TICK 1 // mS

#define CONSUMER_CORE 1
#define CONSUMER_PRIORITY 1
#define CONSUMER_STACKSIZE 8192
#define CONSUMER_TASKNAME "consumer"
#define CONSUMER_TICK 500 // mS

bool initProducerTask(void);
bool initConsummerTask(void);

typedef struct {
  int value;
} buffer_t;

buffer_t initTB = {.value = 0};

TripleBuffer<buffer_t> triple_buffer(initTB);

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
}

void loop() { yield(); }

void producer(void *p) {
  int n = 0;
  while (true) {
    auto &bufferWriteRef = triple_buffer.getWriteRef();
    bufferWriteRef.value = n++;
    triple_buffer.flipWriter();
    delay(PRODUCER_TICK);
  }
}

void consumer(void *p) {
  while (true) {
    triple_buffer.newSnap();
    auto &bufferReadRef = triple_buffer.getReadRef();
    Serial.printf("consume: %d\n", bufferReadRef.value);
    delay(CONSUMER_TICK);
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
