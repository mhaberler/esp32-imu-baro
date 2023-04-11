#include "dps3xx.h"
#include "util.h"

// settings - see datasheet p17 5.1  Measurement Settings and Use Case Examples
// recommended values for:
// Sports (High precision, high rate, background mode)
// PM_PRC = 0x26
//  ..... PM_RATE = 2       4 measurements/second
//  ..... PM_PRC = 6        0110 - 64 times oversampling (High Precision).
//                          note P_SHIFT bit in CFG_REG - must be 1
#define PRESSURE_RATE 2
#define PRESSURE_OVERSAMPLING 6
// TMP_CFG = 0xA0
// ...... TMP_EXT = 1       use external MEMS sensor for temperature
// ...... TMP_RATE = 2      4 measurements/second
// ...... TMP_PRC  = 0      no temperature oversampling
#define TEMPERATURE_RATE 2
#define TEMPERATURE_OVERSAMPLING 0

static Dps368 sensor = Dps368();

bool dps368_i2c_init(TwoWire &bus, uint8_t addr) {

  sensor.begin(bus, addr);
  uint8_t product = sensor.getProductId();
  uint8_t revision = sensor.getRevisionId();

  // dps368: product=0x0 0 rev=0x1 1
  Serial.printf("dps368 at %x : product=%d rev=%d\n", addr, product, revision);

  int16_t n =
      sensor.startMeasureBothCont(TEMPERATURE_RATE, TEMPERATURE_OVERSAMPLING,
                                  PRESSURE_RATE, PRESSURE_OVERSAMPLING);
  if (n != DPS__SUCCEEDED) {
    Serial.printf("startMeasureBothCont fail: 0x%x %d\n", n, n);
    return false;
  }
  return true;
}

bool dps368_i2c_update(JsonDocument &jd, bool temp, bool average) {
  float temperature[DPS__FIFO_SIZE];
  float pressure[DPS__FIFO_SIZE];
  uint8_t tempCount, prsCount;

  uint32_t now = micros();
  int16_t r = sensor.getContResults(temperature, tempCount, pressure, prsCount);

  JsonObject j = jd.createNestedObject("dps368");
  j["ts"] = now;

  if (temp && tempCount) {
    if (!average) {
      JsonArray temps = j.createNestedArray("temps");
      if (temps) {
        for (int i = 0; i < tempCount; i++) {
          temps.add(temperature[i]);
        }
      }
    } else {
      float ta = 0.0;
      for (int i = 0; i < tempCount; i++) {
        ta += temperature[i];
      }
      j["temp"] = ta / tempCount;
    }
  }
  if (prsCount) {
    if (!average) {
      JsonArray press = j.createNestedArray("hpa");
      if (press) {
        for (int i = 0; i < prsCount; i++) {
          press.add(pressure[i] / 100.0);
        }
      }
      JsonArray alts = j.createNestedArray("alt");
      if (alts) {
        for (int i = 0; i < prsCount; i++) {
          alts.add(Pascal2meters(pressure[i], SEALEVELPRESSURE_HPA));
        }
      }
    } else {
      float pa = 0.0;
      float aa = 0.0;
      for (int i = 0; i < prsCount; i++) {
        pa += (pressure[i] / 100.0);
        aa += Pascal2meters(pressure[i], SEALEVELPRESSURE_HPA);
      }
      j["hpa"] = pa / prsCount;
      j["alt"] = aa / prsCount;
    }
  }
  return true;
}
