#include "FlowSensor.hpp"
#include "Ticker.h"
#include "TimerStats.h"
#include "custom.hpp"
#include "defs.hpp"
#include <StreamLib.h>

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
#ifdef FLOWSENSOR_PIN
FlowSensor flow_sensor;
#endif

bool motion_cal;

volatile bool run_imu;

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

  Console.fmt("\n\n--------------------\n");
  platform_report(Console);
  psram_report(Console, __FILE__, __LINE__);
  build_setup_report(Console);
  heap_report(Console, __FILE__, __LINE__);
  Console.fmt("\n\n--------------------\n\n");

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

  Wire.begin();
  Wire.setClock(400000);
  Console.fmt("Wire I2C speed: {}\n", Wire.getClock());

#ifdef HAVE_WIRE1
  Wire1.begin();
  Wire.setClock(400000);
  Console.fmt("Wire1 I2C speed: {}\n", Wire1.getClock());
#endif

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
#ifdef LOOP_PIN
  pinMode(LOOP_PIN, OUTPUT);
  digitalWrite(LOOP_PIN, LOW);
#endif
#ifdef ISR_PIN
  pinMode(ISR_PIN, OUTPUT);
  digitalWrite(ISR_PIN, LOW);
#endif
  Console.fmt("reading stored config: {}\n",
              readPrefs(options) ? "OK" : "FAILED");
  // options.selected_imu = DEV_NONE; // recover
  options.selected_imu_name = imu_devices[options.selected_imu].name;
  customCommands(config, options);
  customInitCode(config, options);
  initShell();

#ifdef WIFI
  WifiSetup(options);
#endif
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
    config.cal.printSavedCalibration();
  }

  initIMU(options, config);
  initOtherwSensors(options, config);

  selectAHRS(options, config);

#ifdef CUSTOM_WATCHDOG_SECONDS
  extern bool loopTaskWDTEnabled;
  loopTaskWDTEnabled = false; // use our own
  watchDogSetup(CUSTOM_WATCHDOG_SECONDS);
#endif

  Console.fmt("starting {} on core {} .. ", sensorTaskName, SENSORTASK_CORE);
  Console.fmt("{}\n", initSensorTask() ? "OK" : "FAILED");

  Console.fmt("starting {} on core {} .. ", reporterTaskName,
              REPORTERTASK_CORE);
  Console.fmt("{}\n", initreporterTask() ? "OK" : "FAILED");

  setStatsRate(0.2);
  setSensorRate(options.imu_rate);
  delay(500); // skip over initial samples which can contain garbage
  setReporterRate(options.report_rate);

#ifdef TEST_LOG
  teleplot.log("startup");
#endif

  // printHelp(options);
}

void loop(void) {
#ifdef LOOP_PIN
  digitalWrite(LOOP_PIN, HIGH);
#endif
  // if (run_imu) {
  //   handleImu(&imu_state);
  //   run_imu = false;
  // }
  // if (run_reporter) {
  //   reporter(&imu_state);
  //   run_reporter = false;
  // }
  if (!motion_cal) {
    testSerial();
  }
#ifdef LOOP_PIN
  digitalWrite(LOOP_PIN, LOW);
#endif
  watchDogRefresh();
  delay(1);
}
