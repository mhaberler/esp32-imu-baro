#pragma once

#define MFID_TPMS1 0x0100
#define TPMS1_PRO_SERVICE_UUID ((uint16_t)0xfbb0)

#define MFID_TPMS2 172

typedef struct {
  float pressure;
  float temperature;
  uint8_t location;
  uint8_t battery; // %
  uint8_t alarm;
} tpms_t;

typedef struct {
  uint16_t mfid;
  uint8_t address[6];
  uint32_t pressure;
  uint32_t temperature;
  uint8_t battery;
  uint8_t alarm;
} tpms1_raw_t;

typedef struct {
  uint16_t mfid;
  uint32_t pressure;
  uint32_t temperature;
  uint8_t battery;
  uint8_t address[6];
} tpms2_raw_t;