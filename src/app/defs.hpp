
#pragma once

#ifdef M5UNIFIED
#include <M5Unified.h>
#endif
#include "defaults.hpp"

#include "freertos-all.h"
#include <ArduinoJson.h>

#include <Adafruit_AHRS.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_LPS2X.h>

#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

#include "espstatus.hpp"
#include "teleplot/Teleplot.h"
#include <Adafruit_BNO08x.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_MPU6886.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Arduino_BMI270_BMM150.h>
#include <TinyGPS++.h>

#include <CmdBuffer.hpp>
#include <CmdCallback.hpp>
#include <CmdParser.hpp>
#include <Ticker.h>

#include "CustomWatchdog.hpp"
extern bool psRAMavail;
#include "ArduinoJsonCustom.hpp"
#include "FS.h"
#include "logmacros.hpp"
#include <FlowSensor.hpp>
#include <Fmt.h>
#include <LittleFS.h>
#include <SparkFunMPU9250-DMP.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <esp_task_wdt.h>
#include <lockless_tripplebuffer/TripleBuffer.h>
#ifdef FASTLED_TYPE
#include <FastLED.h>
#endif
#include "TreeWalker.hpp"
#include "meteo.hpp"

#define B2S(x) ((x) ? "true" : "false")
#define T2OK(x) ((x) ? "OK" : "FAILED")

#define RAD2DEG(r) ((r)*57.29577951f)
#define DEG2RAD(d) ((d)*0.017453292f)
#define G2MG(d) ((d)*1000.)

#define TOGGLE(pin) digitalWrite(pin, !digitalRead(pin))

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define AT __FILE__ ":" TOSTRING(__LINE__)

#define SSID_SIZE 32
#define IPADR_SIZE 32
#define WSPATH_SIZE 32
#define NUM_SSID 10

typedef enum {
  DEV_NONE,
  DEV_FXOS8700_FXAS21002C,
  DEV_MPU6886,
  DEV_MPU9250_DMP,
  DEV_MPU6050,
  DEV_BMI270_BMM150,
  DEV_ICM20948,
  DEV_BNO08x,
  // DEV_SH200Q,
  DEV_ENDMARK = -1
} device_type_t;

typedef struct {
  device_type_t dev;
  const char *name;
} dev_entry_t;
extern dev_entry_t imu_devices[];

typedef enum {
  ACCEL_NONE = 0,
  ACCEL_FXOS8700,
  ACCEL_MPU6886,
  ACCEL_MPU9250_DMP,
  ACCEL_MPU6050,
  ACCEL_SH200Q,
  ACCEL_BMI270,
  ACCEL_ICM20948
} accel_type_t;

typedef enum {
  GYRO_NONE = 0,
  GYRO_FXAS21002C,
  GYRO_MPU6886,
  GYRO_MPU9250_DMP,
  GYRO_MPU6050,
  // GYRO_SH200Q,
  GYRO_BMI270,
  GYRO_ICM20948
} gyro_type_t;

typedef enum {
  MAG_NONE = 0,
  MAG_FXOS8700,
  MAG_BMM150,
  MAG_MPU9250_DMP,
  MAG_ICM20948
} mag_type_t;

typedef enum {
  ALGO_NXP_FUSION = 0,
  ALGO_MADGEWICK,
  ALGO_MAHONEY,
  ALGO_HAR_IN_AIR
} ahrs_algo_t;

// which baro sensor is used as Kalman input?
typedef enum {
  USE_BARO_NONE = 0,
  USE_BARO_LPS22,
  USE_BARO_DPS3XX,
  USE_BARO_BMP3XX,
  // must be last
  USE_BARO_MAX
} use_baro_t;

// this for da blinkenleds
typedef enum {
  ACT_REPORTER,
  ACT_SENSOR,
  ACT_LOOP,
  ACT_IDLEHOOK,
  ACT_TCPSTACK,
  ACT_ENDMARKER
} activitySource_t;

typedef struct {
  float hpa;          // hectoPascal
  float alt;          // meters
  uint32_t timestamp; // mS
} baro_report_t;

typedef struct {
  uint32_t accel_time;
  uint32_t gyro_time;
  uint32_t magnetometer_time;

  uint32_t runtime;          // per IMU read + AHRS update, micros
  uint32_t last_imu_update;  // micros()
  uint32_t last_baro_update; // micros()

  sensors_event_t accel, gyro, mag; // as from drivers, possibly calibrated

  // after optionl North-East-Down translation
  sensors_event_t final_accel, final_gyro, final_mag;
  sensors_event_t temp;
  float roll, pitch, heading;
  float qw, qx, qy, qz;
  float gravx, gravy, gravz;

  baro_report_t baro_values[USE_BARO_MAX];
  flowsensor_report_t flowsensor_values;
} sensor_state_t;

extern TinyGPSPlus serialGps;

typedef struct {
  accel_type_t accel_type;
  const char *accel_name;

  gyro_type_t gyro_type;
  const char *gyro_name;

  mag_type_t magnetometer_type;
  const char *magnetometer_name;

  Adafruit_AHRS_FusionInterface *filter;

  bool dps3xx_avail;
  bool bmp390_avail;
  bool lps22_avail;
  bool fxos8700_avail;
  bool fxas21002_avail;
  bool bno08x_avail;
  bool icm20948_avail;
  bool mpu6050_avail;
  bool mpu9250_dmp_avail;
  bool bmi270_avail;
  bool bmm150_avail;
  bool mpu6886_avail;
  bool flowsensor_avail;
  bool ubloxi2c_avail;
  bool serialgps_avail;
  bool wire_avail, wire1_avail;

  IPAddress tpHost;
  int tpPort;
  struct tm timeinfo;     // from NTP query time
  int64_t timeSinceEpoch; // epoch milliseconds at NTP query time
  unsigned long
      timeinfo_millis; // mS elapsed since boot when NTP time was measured

  Adafruit_Sensor_Calibration_EEPROM cal;
  // gyro calibration offset
  // see
  // https://github.com/adafruit/Adafruit_SensorLab/blob/master/examples/calibration/gyro_zerorate_simplecal/gyro_zerorate_simplecal.ino
  int32_t gcal_samples;
  float min_x, max_x, mid_x;
  float min_y, max_y, mid_y;
  float min_z, max_z, mid_z;

  // misc status
  uint8_t num_websocket_clients;
  uint8_t reporter_stack_util, sensor_stack_util;
  char *txbuf;
  size_t txbuf_size;

  // FS stuff
  bool sd_avail;
  bool lfs_avail;

  SpiRamJsonDocument *root, *client_result;
} config_t; // FIXME should be status_t

extern config_t config;

typedef struct {

  ahrs_algo_t ahrs_algo;
  bool run_filter;
  bool report_raw;

  bool report_hpr;
  bool report_quat;
  bool report_baro;
  bool report_grav;
  bool teleplot_viewer;
  bool apply_cal;
  bool ndjson;
  bool ned;
  bool timing_stats;
  bool memory_usage;

  // HZ
  // < 0: off
  float report_rate;
  float imu_rate;
  float stats_rate;
  device_type_t selected_imu;
  const char *selected_imu_name;
  use_baro_t which_baro;
  uint32_t which_kfmask;

  int flowsensor_pin;

  // smoothing
  float alpha;

  // WiFi
  char ssids[NUM_SSID][SSID_SIZE];
  char passwords[NUM_SSID][SSID_SIZE];
  int8_t num_ssid;
  char hostname[IPADR_SIZE];

  // ntp
  char ntp_poolname[IPADR_SIZE];

  // teleplot
  int32_t tpPort;
  char tpHost[SSID_SIZE];

  // serial GPS
  int32_t gps_uart;
  int32_t gps_speed;
  int32_t gps_rx_pin, gps_tx_pin;

  // debug flag
  int debug;

  // web etc
  bool run_webserver;
  uint16_t webserver_port;
  bool run_webserial;
  bool run_tpwebsocket;

  // FS stuff
  char littlefs_static_path[WSPATH_SIZE];
  char websocket_path[WSPATH_SIZE];
  char consolesocket_path[WSPATH_SIZE];
  char web_default_path[WSPATH_SIZE];
  char web_user[WSPATH_SIZE];
  char web_pass[WSPATH_SIZE];

  // default timers, tasking etc
  uint32_t flush_ms;
  uint32_t reporter_stack;
  uint32_t sensor_stack;
  uint8_t sensor_core;
  uint8_t reporter_core;
  uint32_t sensor_prio;
  uint32_t reporter_prio;

  // SD card
  uint32_t sd_wait_ms;
  int8_t sd_cs_pin;
  uint32_t sd_freq;
  char sd_mountpoint[WSPATH_SIZE];
  uint8_t sd_maxfiles;
  bool sd_format_if_empty;

  // LittleFS
  bool lfs_format_if_empty;
  uint8_t lfs_maxfiles;
  char lfs_mountpoint[WSPATH_SIZE];
  char lfs_partition_label[WSPATH_SIZE];

} options_t;
extern options_t options;

// shell
extern CmdCallback<NUM_COMMANDS> cmdCallback;
extern CmdBuffer<CMD_BUFSIZE> buffer;
extern CmdParser shell;

extern bool motion_cal;
extern volatile bool run_reporter, run_stats, run_sensors;
extern const char *baro_types[];

extern CyclicTask *sensorTask, *reporterTask;
extern const char *sensorTaskName;
extern const char *reporterTaskName;

// teleplot units
extern const char *meter;
extern const char *cm;
extern const char *cps;
extern const char *mps;
extern const char *hpascal;
extern const char *ms2;
extern const char *dps;
extern const char *deg;
extern const char *uT;
extern const char *uS;
extern const char *bool_;
extern const char *bytes_;
extern const char *counter_;
extern const char *percent_;

// Streams
extern Fmt Console, bSerial;
extern Teleplot teleplot;

extern Ticker ubloxStartupTicker, stats_ticker;
void flushBuffers(void);

void initShell(void);
void testSerial(void);
void testWebSerial(void);
void i2cScan(void);

void ublox_setup(void);
bool ublox_detect(const config_t &config, bool debug);
void ublox_loopcheck(void);
void ublox_nav_pvt(UBX_NAV_PVT_data_t *ub);

#include "tinygpsmisc.hpp"

bool readPrefs(options_t &opt);
bool savePrefs(options_t &opt);
void getDefaultPrefs(options_t &opt);
bool wipePrefs(void);

bool selectAHRS(options_t &opt, config_t &config);
void handleSensors(config_t &config, const options_t &options);
void reporter(config_t &config, options_t &opt);
void printSensorsDetected(void);
void printHelp(options_t &options);
void printCurrentCalibration(void);
const char *boardType(void);

const char *algoName(ahrs_algo_t algo);
void MotionCal(sensor_state_t &state, const options_t &options,
               config_t &config);

int WifiSetup(const options_t &opt);
int browseService(const char *service, const char *proto);
void printLocalTime(sensor_state_t *state);
void flushAll(void);
void setStatsRate(const float hz);

void setRate(Ticker &ticker, const float Hz, volatile bool *flag);

void initIMU(options_t &options, config_t &config);
void initOtherSensors(options_t &options, config_t &config);

// webserver
void configureWebServer(const options_t &opt);

// spdlog
void setup_syslog(void);
void set_syslog_loglevel(const int level);
// tasks

#define OPT(x) options->x

extern Adafruit_FXOS8700 fxos;
extern Adafruit_FXAS21002C fxas;

extern Adafruit_Sensor *accelerometer;
extern Adafruit_Sensor *magnetometer;
extern Adafruit_Sensor *gyroscope;

extern Adafruit_BMP3XX bmp;
extern Adafruit_LPS22 lps;
extern Adafruit_DPS310 *dps3xx;

extern Adafruit_Sensor *lps2x_pressure;
extern Adafruit_Sensor *dps3xx_pressure;
extern BoschSensorClass *bmi270_bmm150;
extern Adafruit_ICM20948 *icm20948;
extern Adafruit_BNO08x *bno08x;
extern Adafruit_MPU6050 *mpu6050;
extern MPU9250_DMP *mpu9250_dmp;
extern Adafruit_MPU6886 *mpu6886;
extern FlowSensor flow_sensor;
