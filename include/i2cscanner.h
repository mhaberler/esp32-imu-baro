#ifndef __I2CSCANNER_H__
#define __I2CSCANNER_H__

#include <Arduino.h>
#include <Wire.h>
void i2cScanner(TwoWire *wire, const char *text = "");
#endif