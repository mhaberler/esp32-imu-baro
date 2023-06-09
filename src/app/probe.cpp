#include "defs.hpp"

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

extern SFE_UBLOX_GNSS myGNSS;

// per-device test functions

// caller: automatic - either mpu9250_dmp is non-NULL
// or there's nothing to free
static bool mpu9250_probe(TwoWire *bus, const i2c_probe_t &p) {
    __mpwire    = bus;
    mpu9250_dmp = new MPU9250_DMP();
    bool result = (mpu9250_dmp->begin() == INV_SUCCESS);
    if (!result) delete mpu9250_dmp;
    return result;
}

// caller: alloc bmi270_bmm150, delete on fail
static bool bmm150_probe(TwoWire *bus, const i2c_probe_t &p) {
    return bmi270_bmm150->begin_bmm150(p.addr, bus);
}

static bool bmi270_probe(TwoWire *bus, const i2c_probe_t &p) {
    return bmi270_bmm150->begin_bmi270(p.addr, bus);
}

static bool dps310_probe(TwoWire *bus, const i2c_probe_t &p) {
    return dps3xx->begin_I2C(p.addr, bus);
}

static bool bmp390_probe(TwoWire *bus, const i2c_probe_t &p) {
    return bmp->begin_I2C(p.addr, bus);
}

static bool lps22_probe(TwoWire *bus, const i2c_probe_t &p) {
    return lps->begin_I2C(p.addr, bus);
}

static bool mpu6050_probe(TwoWire *bus, const i2c_probe_t &p) {
    return mpu6050->begin(p.addr, bus);
}

static bool mpu6886_probe(TwoWire *bus, const i2c_probe_t &p) {
    return mpu6886->begin(p.addr, bus);
}

#if defined(DRV_BNO08x)
static bool bno08x_probe(TwoWire *bus, const i2c_probe_t &p) {
    return bno08x->begin_I2C(p.addr, bus);
}
#endif

static bool icm20948_probe(TwoWire *bus, const i2c_probe_t &p) {
    return icm20948->begin_I2C(p.addr, bus);
}

static bool fxos_probe(TwoWire *bus, const i2c_probe_t &p) {
    return fxos->begin(p.addr, bus);
}

static bool fxas_probe(TwoWire *bus, const i2c_probe_t &p) {
    return fxas->begin(p.addr, bus);
}

static bool ublox_probe(TwoWire *bus, const i2c_probe_t &p) {
    return myGNSS.begin(*bus, p.addr);
}

static bool tmp117_probe(TwoWire *bus, const i2c_probe_t &p) {
    tmp117  = new Adafruit_TMP117();
    bool rc = tmp117->begin(p.addr, bus);
    if (!rc) delete tmp117;
    return rc;
}

static bool ina219_probe(TwoWire *bus, const i2c_probe_t &p) {
    ina219  = new Adafruit_INA219(p.addr);
    bool rc = ina219->begin(bus, false);
    if (!rc) delete ina219;
    return rc;
}

// probe arrays are terminated by a I2C_NONE bus  entry
static const i2c_probe_t _bmm150_devs[] = {
    {EXT_I2C, BMM150_DEFAULT_I2C_ADDRESS, bmm150_probe, NULL},
    {EXT_I2C, BMM150_I2C_ADDRESS_CSB_LOW_SDO_HIGH, bmm150_probe, NULL},
    {EXT_I2C, BMM150_I2C_ADDRESS_CSB_HIGH_SDO_LOW, bmm150_probe, NULL},
    {EXT_I2C, BMM150_I2C_ADDRESS_CSB_HIGH_SDO_HIGH, bmm150_probe, NULL},

    {INT_I2C, BMM150_DEFAULT_I2C_ADDRESS, bmm150_probe, NULL},
    {INT_I2C, BMM150_I2C_ADDRESS_CSB_LOW_SDO_HIGH, bmm150_probe, NULL},
    {INT_I2C, BMM150_I2C_ADDRESS_CSB_HIGH_SDO_LOW, bmm150_probe, NULL},
    {INT_I2C, BMM150_I2C_ADDRESS_CSB_HIGH_SDO_HIGH, bmm150_probe, NULL},
    {I2C_NONE, 0, NULL, NULL},
};
const i2c_probe_t *bmm150_devs = &_bmm150_devs[0];

static const i2c_probe_t _bmi270_devs[] = {
    {EXT_I2C, BMI2_I2C_SEC_ADDR, bmi270_probe, NULL},
    {EXT_I2C, BMI2_I2C_PRIM_ADDR, bmi270_probe, NULL},

    {INT_I2C, BMI2_I2C_SEC_ADDR, bmi270_probe, NULL},
    {INT_I2C, BMI2_I2C_PRIM_ADDR, bmi270_probe, NULL},
    
    {I2C_NONE, 0, NULL, NULL},
};
const i2c_probe_t *bmi270_devs = &_bmi270_devs[0];

static const i2c_probe_t _mpu9250_devs[] = {
    {INT_I2C, 0, mpu9250_probe, NULL},
    {EXT_I2C, 0, mpu9250_probe, NULL},
    {I2C_NONE, 0, NULL, NULL},
};
const i2c_probe_t *mpu9250_devs = &_mpu9250_devs[0];

static const i2c_probe_t _dps310_devs[] = {
    {EXT_I2C, DPS3XX_ALTERNATE_ADDRESS, dps310_probe, NULL},
    {INT_I2C, DPS3XX_ALTERNATE_ADDRESS, dps310_probe, NULL},

    {I2C_NONE, 0, NULL, NULL},
};
const i2c_probe_t *dps310_devs = &_dps310_devs[0];

static const i2c_probe_t _bmp390_devs[] = {
    {INT_I2C, BMP3XX_DEFAULT_ADDRESS, bmp390_probe, NULL},
    {EXT_I2C, BMP3XX_DEFAULT_ADDRESS, bmp390_probe, NULL},

    {I2C_NONE, 0, NULL, NULL},
};
const i2c_probe_t *bmp390_devs = &_bmp390_devs[0];

static const i2c_probe_t _lps22_devs[] = {

    {INT_I2C, LPS2X_I2CADDR_DEFAULT, lps22_probe, NULL},
    {INT_I2C, LPS2X_ALTERNATE_ADDRESS, lps22_probe, NULL},
    {EXT_I2C, LPS2X_I2CADDR_DEFAULT, lps22_probe, NULL},
    {EXT_I2C, LPS2X_ALTERNATE_ADDRESS, lps22_probe, NULL},

    {I2C_NONE, 0, NULL, NULL},
};
const i2c_probe_t *lps22_devs = &_lps22_devs[0];

static const i2c_probe_t _mpu6050_devs[] = {
    {INT_I2C, MPU6050_I2CADDR_DEFAULT, mpu6050_probe, NULL},
    {EXT_I2C, MPU6050_I2CADDR_DEFAULT, mpu6050_probe, NULL},

    {I2C_NONE, 0, NULL, NULL},
};
const i2c_probe_t *mpu6050_devs = &_mpu6050_devs[0];

static const i2c_probe_t _mpu6886_devs[] = {
    {INT_I2C, MPU6886_I2CADDR_DEFAULT, mpu6886_probe, NULL},
    {EXT_I2C, MPU6886_I2CADDR_DEFAULT, mpu6886_probe, NULL},

    {I2C_NONE, 0, NULL, NULL},
};
const i2c_probe_t *mpu6886_devs = &_mpu6886_devs[0];

#if defined(DRV_BNO08x)
static const i2c_probe_t _bno08x_devs[] = {
    {INT_I2C, BNO08x_I2CADDR_DEFAULT, bno08x_probe, NULL},
    {EXT_I2C, BNO08x_I2CADDR_DEFAULT, bno08x_probe, NULL},

    {I2C_NONE, 0, NULL, NULL},
};
const i2c_probe_t *bno08x_devs = &_bno08x_devs[0];
#endif

static const i2c_probe_t _icm20948_devs[] = {
    {INT_I2C, ICM20948_I2CADDR_DEFAULT, icm20948_probe, NULL},
    {EXT_I2C, ICM20948_I2CADDR_DEFAULT, icm20948_probe, NULL},

    {I2C_NONE, 0, NULL, NULL},
};
const i2c_probe_t *icm20948_devs = &_icm20948_devs[0];

static const i2c_probe_t _fxos_devs[] = {
    {INT_I2C, 0x1F, fxos_probe, NULL},
    {EXT_I2C, 0x1F, fxos_probe, NULL},

    {I2C_NONE, 0, NULL, NULL},
};
const i2c_probe_t *fxos_devs = &_fxos_devs[0];

static const i2c_probe_t _fxas_devs[] = {
    {INT_I2C, 0x21, fxas_probe, NULL},
    {EXT_I2C, 0x21, fxas_probe, NULL},

    {I2C_NONE, 0, NULL, NULL},
};
const i2c_probe_t *fxas_devs = &_fxas_devs[0];

static const i2c_probe_t _ublox_devs[] = {
    {EXT_I2C, UBLOX_DEFAULT_ADDRESS, ublox_probe, NULL},
    {INT_I2C, UBLOX_DEFAULT_ADDRESS, ublox_probe, NULL},

    {I2C_NONE, 0, NULL, NULL},
};
const i2c_probe_t *ublox_devs = &_ublox_devs[0];

static const i2c_probe_t _tmp117_devs[] = {

    {INT_I2C, TMP117_I2CADDR_DEFAULT, tmp117_probe, NULL},
    {INT_I2C, TMP117_I2CADDR_DEFAULT + 1, tmp117_probe, NULL},

    {EXT_I2C, TMP117_I2CADDR_DEFAULT, tmp117_probe, NULL},
    {EXT_I2C, TMP117_I2CADDR_DEFAULT + 1, tmp117_probe, NULL},

    {I2C_NONE, 0, NULL, NULL},
};
const i2c_probe_t *tmp117_devs = &_tmp117_devs[0];

static const i2c_probe_t _ina219_devs[] = {
    {EXT_I2C, INA219_CALC_ADDRESS(0, 0), ina219_probe, NULL},
    {EXT_I2C, INA219_CALC_ADDRESS(0, 1), ina219_probe, NULL},
    {EXT_I2C, INA219_CALC_ADDRESS(1, 0), ina219_probe, NULL},
    {EXT_I2C, INA219_CALC_ADDRESS(1, 1), ina219_probe, NULL},
#ifndef CORES3  // collides with ES7210 at Wire0 0x40

    {INT_I2C, INA219_CALC_ADDRESS(0, 0), ina219_probe, NULL},
    {INT_I2C, INA219_CALC_ADDRESS(0, 1), ina219_probe, NULL},
    {INT_I2C, INA219_CALC_ADDRESS(1, 0), ina219_probe, NULL},
    {INT_I2C, INA219_CALC_ADDRESS(1, 1), ina219_probe, NULL},
#endif
    {I2C_NONE, 0, NULL, NULL},
};
const i2c_probe_t *ina219_devs = &_ina219_devs[0];
