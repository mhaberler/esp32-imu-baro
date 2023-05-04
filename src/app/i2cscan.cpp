#include "defs.hpp"

#define I2C_MAX 0x78

#ifdef M5_I2CSCAN

#include <M5Unified.h>

void i2cScan(void) {
  Console.fmt("\ninternal I2C devices: ('Wire'):\n");
  bool scan[128];
  M5.In_I2C.scanID(scan);
  for (int i = 8; i < I2C_MAX; ++i) {
    if (scan[i]) {
      Console.fmt("intern: {:#02x}\n", i);
    }
  }
#ifdef SECONDARY_I2C_PORT
  Console.fmt("\nexternal I2C (red port) devices: ('Wire1'):\n");
  M5.Ex_I2C.scanID(scan);
  for (int i = 8; i < I2C_MAX; ++i) {
    if (scan[i]) {
      Console.fmt("extern: {:#02x}\n", i);
    }
  }
#endif
  Console.fmt("\n");
}

#else

#include <Arduino.h>
#include <Wire.h>

void printI2CBusScan(TwoWire &theWire, const char *tag) {
  theWire.begin();
  for (uint8_t addr = 0x8; addr <= I2C_MAX; addr++) {
    theWire.beginTransmission(addr);
    if (theWire.endTransmission() == 0) {
      Console.fmt("{} {:#02x}\n", tag, addr);
    }
  }
  Console.fmt("\n");
}

void i2cScan(void) {
  Console.fmt("\n");
  Console.fmt("Default port (Wire) I2C scan\n");
  printI2CBusScan(DEFAULT_I2C_PORT, "");

#if defined(SECONDARY_I2C_PORT)
  Console.fmt("Secondary port (Wire1) I2C scan\n");
  printI2CBusScan(SECONDARY_I2C_PORT, "";
#endif
  Console.fmt("\n\n");
}
#endif
