#include "defs.hpp"

using namespace m5;
extern options_t options;
extern config_t config;

TwoWire *busToWire(const i2cBus_t bus) {
    // test here for configured bus availability
    switch (bus) {
        case EXT_I2C:
            if (config.i2c_avail[bus]) {
                return &Wire1;
            }
            break;
        case INT_I2C:
            if (config.i2c_avail[bus]) {
                return &Wire;
            }
            break;
        default:
            return NULL;
    }
    return NULL;
}

I2C_Class *busToI2C(const i2cBus_t bus) {
    // test here for configured bus availability
    switch (bus) {
        case EXT_I2C:
            if (config.i2c_avail[bus]) {
                return &M5.Ex_I2C;
            }
            break;
        case INT_I2C:
            if (config.i2c_avail[bus]) {
                return &M5.In_I2C;
            }
            break;
        default:
            return NULL;
    }
    return NULL;
}

bool i2cBusAvailable(const i2cBus_t bus) {
    return busToWire(bus) != NULL;
}

// generic prober
const i2c_probe_t *probe_dev(const char *name, const i2c_probe_t *p) {
    // LOGD("probing: {:<10}", name);
    bool result;
    while (p->bus != I2C_NONE) {
        TwoWire *wire  = busToWire(p->bus);
        I2C_Class *i2c = busToI2C(p->bus);
        if (i2c != NULL) {
            if (!i2c->isEnabled()) {
                i2c->begin();
            }
            result = i2c->start(p->addr, false, 100000) && i2c->stop();
            if (result) {
                LOGD("probing device {:<10} at Wire{} 0x{:X} ...", name,
                     (int)p->bus, (int)p->addr);
                result = p->probe(wire, *p);
            }
            i2c->stop();
            if (result) {
                if (name) {
                    LOGI("device {:<10} found at Wire{} 0x{:X}", name,
                         (int)p->bus, p->addr);
                }
                return p;
            }
        }
        p++;
    }
    LOGI("no {} found.", name);
    return NULL;
}
