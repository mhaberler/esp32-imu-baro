#include "FlowSensor.hpp"
#include "Ticker.h"
#include "TimerStats.h"
#include "custom.hpp"
#include "defs.hpp"
#include "freertos-all.h"
#include <NeoTeeStream.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <StreamLib.h>
#include <StreamUtils.h>
#include <WebSerial.h>

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

bool motion_cal;
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

Ticker flushTicker, ubloxStartupTicker, reporter_ticker, stats_ticker,
    sensor_ticker;
;

Fmt Console(&tee);

CyclicTask *backgroundTask = new CyclicTask(
    "background", BACKGROUNDTASK_STACKSIZE, BACKGROUNDTASK_PRIORITY,
    []() { background(); }, 1000.0 / BACKGROUND_RATE);

CyclicTask *sensorTask = new CyclicTask(
    "fast", SENSORTASK_STACKSIZE, SENSORTASK_PRIORITY,
    []() { handleSensors(config, options); }, 1000.0 / IMU_RATE);

CyclicTask *reporterTask = new CyclicTask(
    "slow", REPORTERTASK_STACKSIZE, REPORTERTASK_PRIORITY,
    []() { reporter(config, options); }, 1000.0 / BACKGROUND_RATE);

void printSensorsDetected(void) {
  Console.fmt("{}{}{}{} a={} g={} m={}\n", config.dps3xx_avail ? "dps3xx " : "",
              config.bmp390_avail ? "bmp3xx " : "",
              config.lps22_avail ? "lps22 " : "",
              config.flowsensor_avail ? "flowsensor" : "", config.accel_name,
              config.gyro_name, config.magnetometer_name);
}

void printCurrentCalibration(void) {
  Console.fmt("accel offsets for zero-g, in m/s^2: {:.4f} {:.4f} {:.4f} \n",
              config.cal.accel_zerog[0], config.cal.accel_zerog[1],
              config.cal.accel_zerog[2]);
  Console.fmt("gyro offsets for zero-rate, in deg/s: {:.4f} {:.4f} {:.4f} \n",
              config.cal.gyro_zerorate[0] * SENSORS_RADS_TO_DPS,
              config.cal.gyro_zerorate[1] * SENSORS_RADS_TO_DPS,
              config.cal.gyro_zerorate[2] * SENSORS_RADS_TO_DPS);
  Console.fmt(
      "mag offsets for hard iron calibration (in uT): {:.4f} {:.4f} {:.4f} \n",
      config.cal.mag_hardiron[0], config.cal.mag_hardiron[1],
      config.cal.mag_hardiron[2]);
  Console.fmt("mag field magnitude (in uT): {:.4f}\n", config.cal.mag_field);
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
    Console.fmt("not implemented yet");
    return NULL;
    break;
  }
  config.filter->begin((int)options.imu_rate);
  opt.run_filter = tmp; // restore previous run_filter setting
  return true;
}

void setup(void) {
#ifdef M5UNIFIED
  auto cfg = M5.config();
  cfg.serial_baudrate =
      BAUD; // default=115200. if "Serial" is not needed, set it to 0.
  cfg.led_brightness = 128; // default= 0. system LED brightness (0=off /
                            // 255=max) (※ not NeoPixel)
  M5.begin(cfg);
#else
  Serial.begin(BAUD);
#endif

  /* Wait for the Serial Monitor */
  while (!Serial) {
    yield();
  }
  Console.fmt("reading stored config: {}\n",
              readPrefs(options) ? "OK" : "FAILED");

  config.serialgps_avail = init_serial_gps(options);

  // start our own background task early so serial flushing works
  // loop() becomes a noop
  // make sure everything called from background() is initalized at this point
  backgroundTask->setRate(1000.0 / options.background_rate);
  backgroundTask->Start(BACKGROUNDTASK_CORE);

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
  Console.fmt("Wire I2C speed: {}\n", Wire.getClock());

  // #ifdef HAVE_WIRE1
  config.wire1_avail = Wire1.begin();
  if (config.wire1_avail) {
    Wire.setClock(400000);
    Console.fmt("Wire1 I2C speed: {}\n", Wire1.getClock());
  } else {
    Console.fmt("Wire1 not available\n");
  }

  // #endif

  Console.fmt("\n\n--------------------\n");
  platform_report(Console);
  Console.fmt("\n\n");
  psram_report(Console, __FILE__, __LINE__);
  Console.fmt("\n\n");
  build_setup_report(Console);
  Console.fmt("\n\n");
  heap_report(Console, __FILE__, __LINE__);
  Console.fmt("\n\n--------------------\n\n");

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

  Console.fmt("board type: {}\n", boardType());

  if (!config.cal.begin()) {
    Console.fmt("Failed to initialize calibration helper\n");
  } else if (!config.cal.loadCalibration()) {
    Console.fmt("No calibration loaded/found\n");
  }
  if (!config.cal.loadCalibration()) {
    Console.fmt("No calibration loaded/found... will start with defaults\n");
  } else {
    Console.fmt("Loaded existing calibration\n");
    // config.cal.printSavedCalibration();
  }

  initOtherSensors(options, config);
  initIMU(options, config);
  selectAHRS(options, config);

  ubloxStartupTicker.once_ms(5000, []() {
    Console.fmt("delayed ublox startup:\n");
    config.ubloxi2c_avail = ublox_detect(config, options.debug > 9);
    Console.fmt("ublox startup ok: {}\n", B2S(config.ubloxi2c_avail));
    if (config.ubloxi2c_avail) {
      ublox_setup();
    }
  });

  customCommands(config, options);
  customInitCode(config, options);
  initShell();

#ifdef WIFI
  WifiSetup(options);
#endif

#if defined(CUSTOM_WATCHDOG_SECONDS) && (CUSTOM_WATCHDOG_SECONDS > 0)
  extern bool loopTaskWDTEnabled;
  loopTaskWDTEnabled = false; // use our own
  watchDogSetup(CUSTOM_WATCHDOG_SECONDS);
#endif

  sensorTask->setRate(1000.0 / options.imu_rate);
  sensorTask->Start(SENSORTASK_CORE);

  reporterTask->setRate(1000.0 / options.report_rate);
  reporterTask->Start(REPORTERTASK_CORE);

  setStatsRate(0.2);

#ifdef TEST_LOG
  teleplot.log("startup");
#endif

  vTaskSuspend(NULL);
}

// make loop exit immediately to destroy "loopTask"
// setup MUST vTaskSuspend(NULL) at the end of setup
void loop(void) { return; }

// the readl backgground task
void background(void) {
#ifdef BACKGROUND_PIN
  digitalWrite(BACKGROUND_PIN, HIGH);
#endif
  if (!motion_cal) {
    testSerial();
  }
  check_serial_gps();
  watchDogRefresh();
  flushBuffers();
#ifdef BACKGROUND_PIN
  digitalWrite(BACKGROUND_PIN, LOW);
#endif
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
  Console.flush();
  bufferedWebSerialOut.flush();
  WebSerial.flush();
  bufferedSerialOut.flush();
}