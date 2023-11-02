#include "defs.hpp"
#include "probe.hpp"
#include <FlowSensor.hpp>
#include <QuadratureDecoder.hpp>

FlowSensor flow_sensor;
QuadratureDecoder quad_sensor;

void initOtherSensors(options_t &options, config_t &config) {
    const i2c_probe_t **dev;
    esp_log_level_t ll = esp_log_level_get("*");
    esp_log_level_set("*", ESP_LOG_ERROR);

    if (options.flowsensor_pin > -1) {
        flow_sensor.begin(options.flowsensor_pin);
        config.flowsensor_avail = flow_sensor.enable();
        LOGD("flowsensor intialized at pin {}: {}", options.flowsensor_pin,
             T2OK(config.flowsensor_avail));
    }
    if ((options.qs_pinA > -1) && (options.qs_pinB > -1)) {
        quad_sensor.begin(options.qs_pinA, options.qs_pinB, options.qs_step,
                          options.qs_tick, options.qs_changeDeltaT,
                          options.qs_inputMode,
                          options.qs_edge);

        config.quad_sensor_avail = quad_sensor.enable();
        LOGD("quadrature sensor intialized at pins {} {}: {}", options.qs_pinA,
             options.qs_pinB, T2OK(config.quad_sensor_avail));
    }

    // try red-port connected sensors before internally wired sensors
    // so they take precedence
    // same for dps310, MUST be at alternate i2c address - collision with
    // bmp3xx
    // very odd bug: works intern or extern depending on order, but not both
    // so delete and recreate sensor object on failure
    dev    = &config.dev[I2C_DPS3XX];
    dps3xx = new Adafruit_DPS310();
    *dev   = probe_dev("dps3xx", dps310_devs);
    if (*dev) {
        // Setup highest precision
        dps3xx->configurePressure(DPS310_64HZ, DPS310_32SAMPLES);
        dps3xx_pressure = dps3xx->getPressureSensor();
        // dps3xx_pressure->printSensorDetails();
    } else {
        delete dps3xx;
    }

    dev = &config.dev[I2C_TMP117];
    // tmp117 = new Adafruit_TMP117();
    *dev = probe_dev("tmp117", tmp117_devs);
    if (*dev) {
        // // Setup highest precision
        // tmp117->configurePressure(DPS310_64HZ, DPS310_32SAMPLES);
        // tmp117 = dps3xx->getPressureSensor();
        // tmp117->printSensorDetails();
    }

    dev = &config.dev[I2C_INA219];
    // ina219 = new Adafruit_INA219();
    *dev = probe_dev("ina219", ina219_devs);
    if (*dev) {
        ina219->setCalibration_32V_2A();
    }

    // bmp390 MUST be at default address 0x77 - collision with dps368 and dps310
    dev  = &config.dev[I2C_BMP390];
    bmp  = new Adafruit_BMP3XX();
    *dev = probe_dev("bmp390", bmp390_devs);
    if (*dev) {
        // FIXME setup?
        // Set up oversampling and filter initialization
        // bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp->setPressureOversampling(BMP3_OVERSAMPLING_32X);
        bmp->setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
        bmp->setOutputDataRate(BMP3_ODR_25_HZ);
    }

    dev  = &config.dev[I2C_LPS22];
    lps  = new Adafruit_LPS22();
    *dev = probe_dev("lps22", lps22_devs);
    if (*dev) {
        lps->setDataRate(LPS22_RATE_25_HZ);
        lps2x_pressure = lps->getPressureSensor();
        // lps2x_pressure->printSensorDetails();
    }
    esp_log_level_set("*", ll);
}
