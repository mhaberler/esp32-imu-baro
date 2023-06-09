#pragma once

#include "stdint.h"

#define MFID_RUUVI 0x0499

typedef struct {
  float temperature;
  float humidity;
  float pressure;
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  float voltage;
  int16_t power;
  uint16_t sequence;
  uint16_t moveCount;
  uint8_t ruuvi_format;
} ruuvi_t;