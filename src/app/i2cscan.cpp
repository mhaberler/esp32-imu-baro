#include "defs.hpp"

#define I2C_MAX 0x78

#include <M5Unified.h>

void i2cScan(void) {
    LOGD("\ninternal I2C devices: ('Wire'):");
    bool scan[128];
    M5.In_I2C.begin();
    M5.In_I2C.scanID(scan);
    M5.In_I2C.release();
    for (int i = 8; i < I2C_MAX; ++i) {
        if (scan[i]) {
            LOGD("intern: {:#02x}", i);
        }
    }
#ifdef SECONDARY_I2C_PORT
    LOGD("\nexternal I2C (red port) devices: ('Wire1'):");
    M5.Ex_I2C.begin();
    M5.Ex_I2C.scanID(scan);
    for (int i = 8; i < I2C_MAX; ++i) {
        if (scan[i]) {
            LOGD("extern: {:#02x}", i);
        }
    }
    M5.Ex_I2C.release();
#endif
}
