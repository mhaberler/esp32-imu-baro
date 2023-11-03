
#include "FlowSensor.hpp"
#include "Ticker.h"
#include "TimerStats.h"
#include "../custom-example/custom.hpp"
#include "defs.hpp"
#include "sdcard.hpp"
#ifdef TREEWALKER
#include "fsVisitor.hpp"
#endif

#include "FunctionalInterrupt.h"
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <NeoTeeStream.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <WebSerial.h>
#include "esp_ota_ops.h"
#include "lv_setup.h"
#if defined(LVGL_UI)
#include "ui/ui.h"
#endif

Adafruit_FXOS8700 *fxos;
Adafruit_FXAS21002C *fxas;

Adafruit_Sensor *accelerometer;
Adafruit_Sensor *magnetometer;
Adafruit_Sensor *gyroscope;

Adafruit_BMP3XX *bmp;
Adafruit_LPS22 *lps;
Adafruit_DPS310 *dps3xx;

Adafruit_Sensor *lps2x_pressure;
Adafruit_Sensor *dps3xx_pressure;
BoschSensorClass *bmi270_bmm150;
Adafruit_ICM20948 *icm20948;
#if defined(DRV_BNO08x)
Adafruit_BNO08x *bno08x;
#endif
Adafruit_MPU6050 *mpu6050;
MPU9250_DMP *mpu9250_dmp;
Adafruit_MPU6886 *mpu6886;
#ifdef DRV_INA219
Adafruit_INA219 *ina219;
#endif
Adafruit_TSL2591 *tsl2591;
INA226 *ina226;
Adafruit_TMP117 *tmp117;

void setSensorRate(const float hz);

bool motion_cal, psRAMavail;
bool flush_buffers = true;

volatile bool run_stats, run_sensors;

options_t options = {
    .selected_imu_name = "<none>",
};

uint32_t last_report;  // micros() of last report

extern TripleBuffer<sensor_state_t> triple_buffer;

config_t config = {
    .accel_type        = ACCEL_NONE,
    .accel_name        = "<none>",
    .gyro_type         = GYRO_NONE,
    .gyro_name         = "<none>",
    .magnetometer_type = MAG_NONE,
    .magnetometer_name = "<none>",
    .gcal_samples      = -1,
    //  {.mux  = portMUX_INITIALIZER_UNLOCKED},
};

// teleplot units
const char *meter    = "m";
const char *cm       = "cm";
const char *cps      = "cm/s";
const char *mps      = "m/s";
const char *hpascal  = "hPa";
const char *ms2      = "m/s^2";
const char *dps      = "deg/s";
const char *deg      = "deg";
const char *uT       = "uT";
const char *uS       = "uS";
const char *mA       = "mA";
const char *volt     = "V";
const char *bool_    = "bool";
const char *bytes_   = "bytes";
const char *counter_ = "counter";
const char *percent_ = "%";

#define WS_BUF  1400
#define LOG_BUF 4096

void background(void);

// WriteBufferingStream bufferedLogger(config.log_fd, LOG_BUF);
WriteBufferingStream *bufferedLogger;

WriteBufferingStream bufferedWebSerialOut(WebSerial, WS_BUF);
Stream *streams[2] = {&Serial, &bufferedWebSerialOut};
NeoTeeStream tee(streams, sizeof(streams) / sizeof(streams[0]));

Fmt Console(&tee);
Fmt serialConsole(&Serial);

CyclicTask *sensorTask, *reporterTask, *bleScanTask;

Ticker flushTicker, logflushTicker, ubloxStartupTicker, reporter_ticker,
    stats_ticker, sensor_ticker;

extern AsyncStaticSdFatWebHandler *handler;
extern DNSServer dnsServer;

static volatile bool run_flush, run_logflush;
bool run_dns;
void handle_deferred(void);

void printSensorsDetected(options_t &options, config_t &config) {
    // LOGD("{}{}{}{} a={} g={} m={}", config.dps3xx_avail ? "dps3xx " : "",
    //      config.bmp390_avail ? "bmp3xx " : "", config.lps22_avail ? "lps22 "
    //      :
    //      "", config.flowsensor_avail ? "flowsensor" : "", config.accel_name,
    //      config.gyro_name, config.magnetometer_name);
}

void printCurrentCalibration(options_t &opt, config_t &config) {
    LOGD("accel offsets for zero-g, in m/s^2: {:.4f} {:.4f} {:.4f}",
         config.cal.accel_zerog[0], config.cal.accel_zerog[1],
         config.cal.accel_zerog[2]);
    LOGD("gyro offsets for zero-rate, in deg/s: {:.4f} {:.4f} {:.4f}",
         config.cal.gyro_zerorate[0] * SENSORS_RADS_TO_DPS,
         config.cal.gyro_zerorate[1] * SENSORS_RADS_TO_DPS,
         config.cal.gyro_zerorate[2] * SENSORS_RADS_TO_DPS);
    LOGD("mag offsets for hard iron calibration (in uT): {:.4f} {:.4f} {:.4f}",
         config.cal.mag_hardiron[0], config.cal.mag_hardiron[1],
         config.cal.mag_hardiron[2]);
    LOGD("mag field magnitude (in uT): {:.4f}", config.cal.mag_field);
}

bool selectAHRS(options_t &opt, config_t &config) {
    // stop the current filter updates during change
    bool tmp       = opt.run_filter;
    opt.run_filter = false;

    switch (opt.ahrs_algo) {
        case ALGO_NXP_FUSION:
            config.filter = new Adafruit_NXPSensorFusion();
            break;
        case ALGO_MADGEWICK:
            config.filter = new Adafruit_Madgwick();
            break;
        case ALGO_MAHONEY:
            config.filter = new Adafruit_Mahony();
            break;
        case ALGO_HAR_IN_AIR:
            LOGD("not implemented yet");
            return NULL;
            break;
    }
    config.filter->begin((int)options.imu_rate);
    opt.run_filter = tmp;  // restore previous run_filter setting
    return true;
}
bool init_globals(void) {
    psRAMavail = ESP.getFreePsram() > 0;
    LOGD("ESP.getFreePsram = {}", ESP.getFreePsram());
    return true;
}

void describe_running_partition(void) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    LOGD("app has started from {:>{}x}", running->address, 4);
}

bool init_littlefs(options_t &opt, config_t &config) {
#ifdef LITTLEFS
    LOGD(
        "LittleFS options: format_on_empty:{}, mountpoint:{} maxfiles:{} "
        "label:{}",
        opt.lfs_format_if_empty, opt.lfs_mountpoint, opt.lfs_maxfiles,
        opt.lfs_partition_label);
    if (LittleFS.begin(opt.lfs_format_if_empty, opt.lfs_mountpoint,
                       opt.lfs_maxfiles, opt.lfs_partition_label)) {
        LOGD("LittleFS mounted, total {}, used {} ({:.1f}%)",
             LittleFS.totalBytes(), LittleFS.usedBytes(),
             100.0 * LittleFS.usedBytes() / LittleFS.totalBytes());
        config.lfs_healthy = true;
        if (opt.max_psram_for_cache_pct > 0) {
// FIXME determine max and apply
#ifdef TREEWALKER
            LOGD("listing LittleFS  toplevel directory:  /");
            fsVisitor(LittleFS, Console, "/", VA_PRINT | VA_DEBUG | VA_CACHE);
#endif
        } else {
            LOGD("LittleFS PSRAM caching disabled");
        }
        return true;
    } else {
        LOGD("LittleFS init failed");
        return false;
    }
#else
    LOGI("LittleFS support not compiled in");
    return false;
#endif
}
bool init_calibration(options_t &opt, config_t &config) {
    config.cal_loaded = false;
    if (!config.cal.begin()) {
        LOGD("Failed to initialize calibration helper");
        return false;
    }
    if (!config.cal.loadCalibration()) {
        LOGD("No calibration loaded/found");
        return false;
    }
    config.cal_loaded = true;
    LOGD("Loaded existing calibration, apply_cal = {}", B2S(opt.apply_cal));
    return true;
}

void setup(void) {
    // #ifdef STARTUP_DELAY
    //     delay(STARTUP_DELAY);  // let USB settle
    // #endif

#ifdef M5UNIFIED
    auto cfg            = M5.config();
    cfg.serial_baudrate = BAUD;
    cfg.led_brightness  = 128;
    cfg.internal_spk    = false;
    cfg.internal_mic    = false;
    cfg.internal_imu    = true;  // enable M5Unified IMU drivers
    cfg.clear_display   = true;

    M5.begin(cfg);
#else
    Serial.begin(BAUD);
#endif

    /* Wait for the Serial Monitor */
    // while (!Serial) {
    //     yield();
    // }

    setup_syslog();
    set_syslog_loglevel(1);
    init_globals();
    describe_running_partition();
    flushBuffers();

    // click power button twice during boot to wipe config
    if (M5.BtnPWR.wasClicked() &&
        (M5.BtnPWR.getClickCount() >= MIN_CLICKS_FOR_WIPE)) {
        Serial.printf("power button clicked - resetting to default config:\n");
        wipePrefs();
        ESP.restart();
    }
    if (Serial.available() && (Serial.read() == 'x')) {
        Serial.printf("x received - resetting to default config:\n");
        wipePrefs();
        ESP.restart();
    }
    lv_begin();
#if defined(LVGL_UI)
    ui_init();
#endif

    bool cfg_read = readPrefs(options);
    if (!cfg_read) {
        wipePrefs();
        getDefaultPrefs(options);
        savePrefs(options);
        cfg_read = readPrefs(options);
        LOGW("options reset to defauls: {}", T2OK(cfg_read));
    }
    config.boot_count = readBootCount();
    LOGI("boot count: {}", config.boot_count);
    LOGI("WDT reset count: {}", readWdtCount());

    extern bool loopTaskWDTEnabled;
    loopTaskWDTEnabled = false;  // use our own, or none if opt.watchdog_ms == 0
    watchDogSetup(options);

    flushBuffers();

    init_sd(options, config);
    if (options.log_to_sd) {
        config.log_open = sd_openlog(options, config);
        if (config.log_open) {
            setRate(logflushTicker, options.log_commit_freq, &run_logflush);
        }
    }

    flushBuffers();
    init_littlefs(options, config);

    flushBuffers();
    if (cfg_read) {
        // /* Set severity for esp logging system. */
        // esp_log_level_set("*", CONFIG_ESP_LOG_SEVERITY);
        // esp_log_level_set("*", ESP_LOG_WARN);
        set_syslog_loglevel(options.log_level);
    }
    flushBuffers();
    LOGD("reading stored config: {}", cfg_read ? "OK" : "FAILED");
#ifdef SERIAL_GPS
    config.serialgps_avail = init_serial_gps(options);
#endif

    i2c_cfg_t *ip = &options.i2c_cfg[0];
    if (ip->kHz > 0) {
        LOGD("In_I2C/Wire params: sda={} scl={} speed={} port={}", ip->sda,
             ip->scl, ip->kHz, (int)M5.In_I2C.getPort());
        M5.In_I2C.begin((i2c_port_t)0, ip->sda, ip->scl);

        if (M5.In_I2C.getPort()) {
            config.i2c_avail[0] =
                Wire1.begin(ip->sda, ip->scl, ip->kHz * 10000);
        } else {
            config.i2c_avail[0] = Wire.begin(ip->sda, ip->scl, ip->kHz * 10000);
        }
    }
    LOGD("In_I2C/Wire available: {}, actual speed: {}",
         B2S(config.i2c_avail[0]), Wire.getClock());

    ip = &options.i2c_cfg[1];
    if (ip->kHz > 0) {
        LOGD("Ex_I2C/Wire1 params: sda={} scl={} speed={} port={}", ip->sda,
             ip->scl, ip->kHz, (int)M5.Ex_I2C.getPort());
        M5.Ex_I2C.begin((i2c_port_t)1, ip->sda, ip->scl);
        if (M5.Ex_I2C.getPort()) {
            config.i2c_avail[1] =
                Wire1.begin(ip->sda, ip->scl, ip->kHz * 10000);
        } else {
            config.i2c_avail[1] = Wire.begin(ip->sda, ip->scl, ip->kHz * 10000);
        }
    }
    LOGD("Ex_I2C/Wire1 available: {}, actual speed: {}",
         B2S(config.i2c_avail[1]), Wire1.getClock());

    flushBuffers();
    platform_report();
    psram_report(__FILE__, __LINE__);
    build_setup_report();

    init_timingpins();

    // options.selected_imu = DEV_NONE; // recover
    options.selected_imu_name = imu_devices[options.selected_imu].name;

    LOGD("board type: {}", boardType());

    if (init_calibration(options, config)) {
        printCurrentCalibration(options, config);
    }
    flushBuffers();
    initOtherSensors(options, config);
    flushBuffers();
    if (options.selected_imu != DEV_NONE) {
        initIMU(options, config);
        flushBuffers();
        selectAHRS(options, config);
    }
    flushBuffers();
#ifdef UBLOX_GPS
    ubloxStartupTicker.once_ms(UBLOX_STARTUP_DELAY, []() {
        LOGD("delayed ublox startup:");
        if (ublox_detect(options, config)) {
            ublox_setup();
        }
    });
#endif
    flushBuffers();
    customCommands(config, options);
    flushBuffers();
    customInitCode(config, options);
    flushBuffers();
    initShell();
    flushBuffers();

#ifdef WIFI
    WifiSetup(options, config);
    flushBuffers();
#ifdef WEBSERVER
    configureWebServer(options, config);
    flushBuffers();
#endif

#endif
    reporterTask = new CyclicTask(
        "reporter", options.reporter_stack, options.reporter_prio,
        []() { reporter(config, options); }, 1000.0 / REPORT_RATE);

    reporterTask->setRate(1000.0 / options.report_rate);
    if (!reporterTask->Start(options.reporter_core)) {
        LOGE("failed to start reporter task!");
        heap_report(NULL, 0);
    }
    sensorTask = new CyclicTask(
        "sensor", options.sensor_stack, options.sensor_prio,
        []() { handleSensors(); }, 1000.0 / IMU_RATE);

    sensorTask->setRate(1000.0 / options.imu_rate);
    if (!sensorTask->Start(options.sensor_core)) {
        LOGE("failed to start sensor task!");
        heap_report(__FILE__, __LINE__);
    }

    setStatsRate(0.2);
    setSensorRate(options.sensor_rate);

#if defined(BLE_DECODER)
    if (options.ble_rate > 0) {
        setupBLE(options, config);
        BLEscanOnce(options, config);
    }
#endif

    task_report(__FILE__, __LINE__, 2, "sensor", "reporter", NULL);
    heap_report(__FILE__, __LINE__);

#ifdef TEST_LOG
    teleplot.log("startup");
#endif
    flushBuffers();
    // switch to just setting a flag
    flushTicker.attach_ms(options.flush_ms, []() { run_flush = true; });
}

void background(void) {
#ifdef BACKGROUND_PIN
    digitalWrite(BACKGROUND_PIN, HIGH);
#endif

    // FIXME wsclients cleanup!!
#ifdef WIFI
    if (run_dns) dnsServer.processNextRequest();
#endif
#ifdef WEBSERVER
    handle_deferred();
#endif
    if (!motion_cal) {
        testSerial();
    }
#ifdef SERIAL_GPS
    check_serial_gps();
#endif
    if (run_flush) {
        flushBuffers();
        run_flush = false;
    }
#ifdef BLE_SUPPORT
    extern Queue bleSensors;
    blesensor_report_t ble;
    while (bleSensors.Dequeue(&ble, 0)) {
        if (options.trace & INFO_BLE_DEQUEUED) {
            print_ble_report(ble);
        }
    }
#endif
    if (run_logflush && bufferedLogger) {
        bufferedLogger->flush();
        run_logflush = false;
    }
    // click power button twice during boot to wipe config
    // M5.update();
    // if (M5.BtnPWR.wasClicked() && (M5.BtnPWR.getClickCount() >=
    // MIN_CLICKS_FOR_WIPE)) {
    //     Serial.printf("resetting to default config:\n");
    //     wipePrefs();
    //     ESP.restart();
    // }

    // lv_handler();

#ifdef BACKGROUND_PIN
    digitalWrite(BACKGROUND_PIN, LOW);
#endif
}

void loop(void) {
    watchDogRefresh();
    background();
    yield();
}

void setSensorRate(const float hz) {
    setRate(sensor_ticker, hz, &run_sensors);
}

void setReporterRate(const float hz) {
    setRate(reporter_ticker, hz, &run_reporter);
}
void setStatsRate(const float hz) {
    setRate(stats_ticker, hz, &run_stats);
}

static void setter(volatile bool *flag) {
    *flag = true;
}

void setRate(Ticker &ticker, const float Hz, volatile bool *flag) {
    if (ticker.active()) {
        ticker.detach();
    }
    ticker.attach(1.0 / Hz, setter, flag);
}

void flushBuffers() {
    // this should be thread-locked with Console.write
    Console.flush();
    bufferedWebSerialOut.flush();
#ifdef WEBSERIAL
    WebSerial.flush();
#endif
    watchDogRefresh();
}