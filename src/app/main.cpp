#include "FlowSensor.hpp"
#include "Ticker.h"
#include "TimerStats.h"
#include "custom.hpp"
#include "defs.hpp"
#include "fsVisitor.hpp"

#include <LittleFS.h>
#include <NeoTeeStream.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <StreamLib.h>
#include <StreamUtils.h>
#include <WebSerial.h>

#ifdef SD_SUPPORT
#include <SD.h>
#endif
#define MARK                                                                   \
  { Serial.printf("mark: %s:%d\n", __FILE__, __LINE__); }

Adafruit_FXOS8700 fxos = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C fxas = Adafruit_FXAS21002C(0x0021002C);

Adafruit_Sensor *accelerometer;
Adafruit_Sensor *magnetometer;
Adafruit_Sensor *gyroscope;

Adafruit_BMP3XX bmp;
Adafruit_LPS22 lps;
Adafruit_DPS310 *dps3xx;

Adafruit_Sensor *lps2x_pressure;
Adafruit_Sensor *dps3xx_pressure;
BoschSensorClass *bmi270_bmm150;
Adafruit_ICM20948 *icm20948;
Adafruit_BNO08x *bno08x;
Adafruit_MPU6050 *mpu6050;
MPU9250_DMP *mpu9250_dmp;
Adafruit_MPU6886 *mpu6886;
FlowSensor flow_sensor;

bool motion_cal, psRAMavail;
bool flush_buffers = true;

volatile bool run_stats;

options_t options = {
    .selected_imu_name = "<none>",
};

uint32_t last_report; // micros() of last report

extern TripleBuffer<sensor_state_t> triple_buffer;

config_t config = {.accel_type = ACCEL_NONE,
                   .accel_name = "<none>",
                   .gyro_type = GYRO_NONE,
                   .gyro_name = "<none>",
                   .magnetometer_type = MAG_NONE,
                   .magnetometer_name = "<none>",
                   .wire_avail = false,
                   .wire1_avail = false,
                   .gcal_samples = -1};

// teleplot units
const char *meter = "m";
const char *cm = "cm";
const char *cps = "cm/s";
const char *mps = "m/s";
const char *hpascal = "hPa";
const char *ms2 = "m/s^2";
const char *dps = "deg/s";
const char *deg = "deg";
const char *uT = "uT";
const char *uS = "uS";
const char *bool_ = "bool";
const char *bytes_ = "bytes";
const char *counter_ = "counter";
const char *percent_ = "%";

#define WS_BUF 1400
#define SERIAL_BUF 512

void background(void);

WriteBufferingStream bufferedSerialOut(Serial, SERIAL_BUF);
WriteBufferingStream bufferedWebSerialOut(WebSerial, WS_BUF);
Stream *streams[2] = {&bufferedSerialOut, &bufferedWebSerialOut};
NeoTeeStream tee(streams, sizeof(streams) / sizeof(streams[0]));

Fmt Console(&tee);
Fmt bSerial(&bufferedSerialOut);

CyclicTask *sensorTask, *reporterTask;

Ticker flushTicker, ubloxStartupTicker, reporter_ticker, stats_ticker,
    sensor_ticker;

static volatile bool run_flush;

void printSensorsDetected(void) {
  LOGD("{}{}{}{} a={} g={} m={}", config.dps3xx_avail ? "dps3xx " : "",
       config.bmp390_avail ? "bmp3xx " : "", config.lps22_avail ? "lps22 " : "",
       config.flowsensor_avail ? "flowsensor" : "", config.accel_name,
       config.gyro_name, config.magnetometer_name);
}

void printCurrentCalibration(void) {
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
  bool tmp = opt.run_filter;
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
  opt.run_filter = tmp; // restore previous run_filter setting
  return true;
}
bool init_globals(void) {
  psRAMavail = ESP.getFreePsram() > 0;
  LOGD("ESP.getFreePsram = {}", ESP.getFreePsram());
  return true;
}

void read_partitions(void) {
  // bool EspClass::flashRead(uint32_t offset, uint32_t *data, size_t
  // size)
  //   ESP.flashRead
  LOGD("ESP32 Partition table:");

  LOGD("| Type | Sub |  Offset  |   Size   |       Label      |");
  LOGD("| ---- | --- | -------- | -------- | ---------------- |");

  esp_partition_iterator_t pi = esp_partition_find(
      ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
  if (pi != NULL) {
    do {
      const esp_partition_t *p = esp_partition_get(pi);
      LOGD("| {:>{}x} | {:>{}x} | {:>{}x} | {:>{}x} | {:<16} |", (int)p->type,
           4, (int)p->subtype, 3, p->address, 8, p->size, 8, (char *)p->label);
    } while ((pi = (esp_partition_next(pi))));
  }
}

bool init_sd(options_t &opt, config_t &config) {
#ifdef SD_SUPPORT
#ifdef M5UNIFIED
  if (opt.sd_cs_pin < 0) {
    LOGI("SD support disabled via config.");
    return false;
  }
  uint32_t start = millis();
  while (false == SD.begin(opt.sd_cs_pin, SPI, opt.sd_freq, opt.sd_mountpoint,
                           5, opt.sd_format_if_empty)) {
    delay(500);
    if ((millis() - start) > opt.sd_wait_ms) {
      LOGI("giving up on SD.");
      return false;
    }
    LOGD("SD waiting... {.f}s", (millis() - start) / 1000.0);
  }
  LOGD("SD mounted.");
  LOGD("SD toplevel directory:");
  fsVisitor(SD, Console, "/", VA_PRINT);
  return true;
#else
  // if (!SD.begin("/sdcard", true)) {
  //   LOGD("SD Mount Failed");
  // }
#endif
#else
  LOGI("SD support not compiled in");
  return false;
#endif
}

bool init_littlefs(options_t &opt, config_t &config) {
#ifdef LITTLEFS_SUPPORT

  if (LittleFS.begin(opt.lfs_format_if_empty, opt.lfs_mountpoint,
                     opt.lfs_maxfiles, opt.lfs_partition_label)) {
    LOGD("LittleFS  total {}, used {} ({:.1f}%)", LittleFS.totalBytes(),
         LittleFS.usedBytes(),
         100.0 * LittleFS.usedBytes() / LittleFS.totalBytes());
    LOGD("listing LittleFS  toplevel directory:  /");

    fsVisitor(LittleFS, Console, "/", VA_PRINT | VA_DEBUG | VA_CACHE);

    return true;
  } else {
    LOGD("LittleFS init failed - formatting and reboot");
    LittleFS.format();
    ESP.restart();
  }
#else
  LOGI("LittleFS support not compiled in");
  return false;
#endif
}

void setup(void) {
#ifdef M5UNIFIED
  auto cfg = M5.config();
  cfg.serial_baudrate =
      BAUD; // default=115200. if "Serial" is not needed, set it to 0.
  cfg.led_brightness = 128; // default= 0. system LED brightness (0=off /
                            // 255=max) (※ not NeoPixel)
                            // cfg.internal_mic = false;
  cfg.internal_spk = false;

  M5.begin(cfg);

  // M5.Display.startWrite();
  // M5.Display.setCursor(0, 0);
  // M5.Display.print("REC");
  // M5.Speaker.setVolume(255);

  // M5.Mic.begin();

#else
  Serial.begin(BAUD);
#endif

  /* Wait for the Serial Monitor */
  while (!Serial) {
    yield();
  }

  setup_syslog();
  set_syslog_loglevel(1);
  init_globals();
  read_partitions();
  flushBuffers();

  bool cfg_read = readPrefs(options);
  if (!cfg_read) {
    wipePrefs();
    getDefaultPrefs(options);
    savePrefs(options);
    cfg_read = readPrefs(options);
    LOGW("options reset to defauls: {}", T2OK(cfg_read));
  }
  flushBuffers();
  init_sd(options, config);
  flushBuffers();
  init_littlefs(options, config);
  flushBuffers();
  if (cfg_read) {

    // /* Set severity for esp logging system. */
    // esp_log_level_set("*", CONFIG_ESP_LOG_SEVERITY);
    // esp_log_level_set("*", ESP_LOG_WARN);
    set_syslog_loglevel(options.debug);
  }
  flushBuffers();
  LOGD("reading stored config: {}", cfg_read ? "OK" : "FAILED");

  config.serialgps_avail = init_serial_gps(options);

  // set I2C clock to 400kHz
  // running out of time in handleImu() with 100kHz
  // #ifdef M5UNIFIED
  // #define I2C_CLOCK_SPEED 100000
  // #else
  // #define I2C_CLOCK_SPEED 400000
  // #endif
  //   // 400kHz is too fast for the extern red port on the Core2
  //   Wire.begin(21, 22, I2C_CLOCK_SPEED);
  // #ifdef HAVE_WIRE1
  //   Wire1.begin(32, 33, I2C_CLOCK_SPEED);
  // #endif
  config.wire_avail = Wire.begin();
  Wire.setClock(400000);
  LOGD("Wire I2C speed: {}", Wire.getClock());

  // #ifdef HAVE_WIRE1
  config.wire1_avail = Wire1.begin();
  if (config.wire1_avail) {
    Wire.setClock(400000);
    LOGD("Wire1 I2C speed: {}", Wire1.getClock());
  } else {
    LOGD("Wire1 not available");
  }
  flushBuffers();
  platform_report();
  psram_report(__FILE__, __LINE__);
  build_setup_report();
  heap_report(__FILE__, __LINE__);

  // timing pins
#ifdef IMU_PIN
  pinMode(IMU_PIN, OUTPUT);
  digitalWrite(IMU_PIN, LOW);
#endif
#ifdef REPORT_PIN
  pinMode(REPORT_PIN, OUTPUT);
  digitalWrite(REPORT_PIN, LOW);
#endif
#ifdef EXTRA_PIN
  pinMode(EXTRA_PIN, OUTPUT);
  digitalWrite(EXTRA_PIN, LOW);
#endif
#ifdef BACKGROUND_PIN
  pinMode(BACKGROUND_PIN, OUTPUT);
  digitalWrite(BACKGROUND_PIN, LOW);
#endif
#ifdef ISR_PIN
  pinMode(ISR_PIN, OUTPUT);
  digitalWrite(ISR_PIN, LOW);
#endif
#ifdef UBLOX_PIN
  pinMode(UBLOX_PIN, OUTPUT);
  digitalWrite(UBLOX_PIN, LOW);
#endif
  // options.selected_imu = DEV_NONE; // recover
  options.selected_imu_name = imu_devices[options.selected_imu].name;

  LOGD("board type: {}", boardType());

  if (!config.cal.begin()) {
    LOGD("Failed to initialize calibration helper");
  } else if (!config.cal.loadCalibration()) {
    LOGD("No calibration loaded/found");
  }
  if (!config.cal.loadCalibration()) {
    LOGD("No calibration loaded/found... will start with defaults");
  } else {
    LOGD("Loaded existing calibration");
    // config.cal.printSavedCalibration();
  }
  flushBuffers();
  initOtherSensors(options, config);
  initIMU(options, config);
  selectAHRS(options, config);

  ubloxStartupTicker.once_ms(UBLOX_STARTUP_DELAY, []() {
    LOGD("delayed ublox startup:");
    config.ubloxi2c_avail = ublox_detect(config, options.debug > 9);
    LOGD("ublox startup ok: {}", B2S(config.ubloxi2c_avail));
    if (config.ubloxi2c_avail) {
      ublox_setup();
    }
  });
  flushBuffers();
  customCommands(config, options);
  customInitCode(config, options);
  initShell();
  flushBuffers();
#ifdef WIFI
  WifiSetup(options);
#endif

#if defined(CUSTOM_WATCHDOG_SECONDS) && (CUSTOM_WATCHDOG_SECONDS > 0)
  extern bool loopTaskWDTEnabled;
  loopTaskWDTEnabled = false; // use our own
  watchDogSetup(CUSTOM_WATCHDOG_SECONDS);
#endif

  sensorTask = new CyclicTask(
      "sensor", options.sensor_stack, options.sensor_prio,
      []() { handleSensors(config, options); }, 1000.0 / IMU_RATE);

  reporterTask = new CyclicTask(
      "reporter", options.reporter_stack, options.reporter_prio,
      []() { reporter(config, options); }, 1000.0 / BACKGROUND_RATE);

  sensorTask->setRate(1000.0 / options.imu_rate);
  sensorTask->Start(options.sensor_core);

  reporterTask->setRate(1000.0 / options.report_rate);
  reporterTask->Start(options.reporter_core);

  setStatsRate(0.2);

  task_report(__FILE__, __LINE__, 2, "sensor", "reporter", NULL);

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
  if (!motion_cal) {
    testSerial();
  }
  check_serial_gps();
  watchDogRefresh();
  if (run_flush) {
    flushBuffers();
    run_flush = false;
  }
#ifdef BACKGROUND_PIN
  digitalWrite(BACKGROUND_PIN, LOW);
#endif
}
void loop(void) {
  background();
  delay(50);
}

void setSensorRate(const float hz) { setRate(sensor_ticker, hz, &run_sensors); }

void setReporterRate(const float hz) {
  setRate(reporter_ticker, hz, &run_reporter);
}
void setStatsRate(const float hz) { setRate(stats_ticker, hz, &run_stats); }

static void setter(volatile bool *flag) { *flag = true; }

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
  WebSerial.flush();
  bufferedSerialOut.flush();
}