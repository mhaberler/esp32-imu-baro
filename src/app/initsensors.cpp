#include "FlowSensor.hpp"
#include "defs.hpp"

void initOtherwSensors(options_t &options, config_t &config) {

#ifdef FLOWSENSOR_PIN
  if (options.flowsensor_pin > -1) {
    flow_sensor.begin(options.flowsensor_pin);
    flow_sensor.enable();
    config.flowsensor_avail = true;
  }
#endif

  // try red-port connected sensors before internally wired sensors
  // so they take precedence
  // same for dps310, MUST be at alternate i2c address - collision with
  // bmp3xx

  // very odd bug: works intern or extern depending on order, but not both
  // so delete and recreate sensor object on failure
#ifdef HAVE_WIRE1
  dps3xx = new Adafruit_DPS310();
  config.dps3xx_avail = dps3xx->begin_I2C(DPS3XX_ALTERNATE_ADDRESS, &Wire1);
  if (!config.dps3xx_avail) {
    delete dps3xx;
    dps3xx = new Adafruit_DPS310();
    config.dps3xx_avail = dps3xx->begin_I2C(DPS3XX_ALTERNATE_ADDRESS, &Wire);
  }
#else
  dps3xx = new Adafruit_DPS310();
  config.dps3xx_avail = dps3xx->begin_I2C(DPS3XX_ALTERNATE_ADDRESS, &Wire);
#endif
  if (config.dps3xx_avail) {
    // Setup highest precision
    dps3xx->configurePressure(DPS310_64HZ, DPS310_32SAMPLES);
    dps3xx_pressure = dps3xx->getPressureSensor();
    dps3xx_pressure->printSensorDetails();
  }

  // bmp390 MUST be at default address 0x77 - collision with dps368 and dps310
#ifdef HAVE_WIRE1
  config.bmp390_avail = bmp.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire1);
  if (!config.bmp390_avail) {
    config.bmp390_avail = bmp.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire);
  }
#else
  config.bmp390_avail = bmp.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire);

#endif

  if (config.bmp390_avail) {
    // FIXME setup?
    // Set up oversampling and filter initialization
    // bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
    bmp.setOutputDataRate(BMP3_ODR_25_HZ);
    Console.fmt("BMP3XX found\n");
  }

#ifdef HAVE_WIRE1
  // try red port first, then internal, any address
  config.lps22_avail = lps.begin_I2C(LPS2X_I2CADDR_DEFAULT, &Wire1);
  if (!config.lps22_avail) {
    config.lps22_avail = lps.begin_I2C(LPS2X_ALTERNATE_ADDRESS, &Wire1);
  }
#endif
  if (!config.lps22_avail) {
    config.lps22_avail = lps.begin_I2C(LPS2X_I2CADDR_DEFAULT, &Wire);
  }
  if (!config.lps22_avail) {
    config.lps22_avail = lps.begin_I2C(LPS2X_ALTERNATE_ADDRESS, &Wire);
  }
  if (config.lps22_avail) {
    lps.setDataRate(LPS22_RATE_25_HZ);
    lps2x_pressure = lps.getPressureSensor();
    lps2x_pressure->printSensorDetails();
  }
  init_serial_gps(options);
}
