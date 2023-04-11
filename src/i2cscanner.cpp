#include "i2cscanner.h"

void i2cScanner(TwoWire *wire, const char *text) {
  Serial.printf("I2C scanner. Scanning bus %s\n", text);
  byte count = 0;

  for (byte i = 8; i < 125; i++) {
    wire->beginTransmission(i);       // Begin I2C transmission Address (i)
    if (wire->endTransmission() == 0) // Receive 0 = success (ACK response)
    {
      Serial.printf("found i2c device at 0x%x\n", i);
      count++;
    }
  }
  Serial.printf("Found %d devices on %s\n", count, text);
}
