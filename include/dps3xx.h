
#ifndef __DPS3XX_H__
#define __DPS3XX_H__

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Dps368.h>
#include <Wire.h>
#define DPS368_DEFAULT_ADDRESS 0x77
#define DPS368_ALTERNATE_ADDRESS 0x76 // for 0x76 pull down INT/GPIO3 pin

bool dps368_i2c_init(TwoWire &bus, uint8_t addr = DPS368_DEFAULT_ADDRESS);
bool dps368_i2c_update(JsonDocument &jd, bool temp = false,
                       bool average = false);
#endif