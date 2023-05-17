#include "defs.hpp"

#define I2C_MAX 0x78

#ifdef M5_I2CSCAN

#include <M5Unified.h>

void i2cScan(void) {
  LOGD("\ninternal I2C devices: ('Wire'):");
  bool scan[128];
  M5.In_I2C.scanID(scan);
  for (int i = 8; i < I2C_MAX; ++i) {
    if (scan[i]) {
      LOGD("intern: {:#02x}", i);
    }
  }
#ifdef SECONDARY_I2C_PORT
  LOGD("\nexternal I2C (red port) devices: ('Wire1'):");
  M5.Ex_I2C.scanID(scan);
  for (int i = 8; i < I2C_MAX; ++i) {
    if (scan[i]) {
      LOGD("extern: {:#02x}", i);
    }
  }
#endif
  LOGD("");
}

#else

#include <Arduino.h>
#include <Wire.h>

void printI2CBusScan(TwoWire &theWire, const char *tag) {
  theWire.begin();
  for (uint8_t addr = 0x8; addr <= I2C_MAX; addr++) {
    theWire.beginTransmission(addr);
    if (theWire.endTransmission() == 0) {
      LOGD("{} {:#02x}", tag, addr);
    }
  }
  LOGD("");
}

void i2cScan(void) {
  LOGD("");
  LOGD("Default port (Wire) I2C scan");
  printI2CBusScan(DEFAULT_I2C_PORT, "");

#if defined(SECONDARY_I2C_PORT)
  LOGD("Secondary port (Wire1) I2C scan");
  printI2CBusScan(SECONDARY_I2C_PORT, "";
#endif
  LOGD("\n");
}
#endif
