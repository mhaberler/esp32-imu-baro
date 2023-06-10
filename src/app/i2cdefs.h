#pragma once

#include "Wire.h"

#define NUM_I2C 2

typedef enum {
    I2C_NULLDEV,
    I2C_LPS22,
    I2C_DPS3XX,
    I2C_BMP390,
    I2C_FXOS,
    I2C_FXAS,
    I2C_BNO08X,
    I2C_ICM20949,
    I2C_MPU6050,
    I2C_MPU9250,
    I2C_BMI270,
    I2C_BMM150,
    I2C_MPU6886,
    I2C_UBLOXI2C,
    I2C_TMP117,
    I2C_INA219,
    I2C_MAX,
} i2c_dev_t;
typedef enum { INT_I2C = 0, EXT_I2C = 1, I2C_NONE } i2cBus_t;

typedef struct i2c_probe i2c_probe_t;
typedef bool (*probe_func)(TwoWire *bus, const i2c_probe_t &entry);

typedef struct i2c_probe {
    const i2cBus_t bus;
    const uint8_t addr;
    const probe_func probe;
    const void *user_data;
} i2c_probe_t;

TwoWire &i2cBus(const i2cBus_t bus);
bool i2cBusAvailable(const i2cBus_t bus);
TwoWire *busToWire(const i2cBus_t bus);
bool i2cBusAvailable(const i2cBus_t bus);
const i2c_probe_t *probe_dev(const char *name, const i2c_probe_t *p);