
#pragma once

#include "defaults.hpp"

#ifndef CONFIG_SPIRAM_SUPPORT
#ifdef BOARD_HAS_PSRAM
#define CONFIG_SPIRAM_SUPPORT
#endif
#endif

#include "FS.h"

#ifdef SDFAT
#include "SdFat.h"
#include "SdFatConfig.h"
#define SD_SCK_KHZ(maxkHz) (1000UL * (maxkHz))
#endif

#ifdef M5UNIFIED_I2C
#include "utility/I2C_Class.hpp"
#endif

#ifdef M5UNIFIED
#include <M5GFX.h>
#include <M5Unified.h>
#endif

#include "freertos-all.h"
#include <esp_task_wdt.h>

#include "espstatus.hpp"
#include <CmdBuffer.hpp>
#include <CmdCallback.hpp>
#include <CmdParser.hpp>
#include <Ticker.h>

#include "i2cdefs.h"
#include "logmacros.hpp"
#include "maskbits.h"
#include "meteo.hpp"

#include <ArduinoJson.h>
#include "ArduinoJsonCustom.hpp"
#include <Fmt.h>
#include <StreamUtils.h>
#include <lockless_tripplebuffer/TripleBuffer.h>

#ifdef TELEPLOT
#include "teleplot/Teleplot.h"
#endif

#ifdef ADAFRUIT_AHRS
#include <Adafruit_AHRS.h>
#endif

#ifdef DRV_BMP3XX
#include <Adafruit_BMP3XX.h>
#endif

#ifdef DRV_DPS310
#include <Adafruit_DPS310.h>
#endif

#ifdef DRV_LPS22
#include <Adafruit_LPS2X.h>
#endif

#ifdef DRV_NXP
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#endif

#ifdef DRV_BNO08x
#include <Adafruit_BNO08x.h>
#endif

#ifdef DRV_ICM20948
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#endif

#ifdef DRV_INA219
#include <Adafruit_INA219.h>
#endif

#ifdef DRV_MPU6050
#include <Adafruit_MPU6050.h>
#endif

#ifdef DRV_MPU6886
#include <Adafruit_MPU6886.h>
#endif

#ifdef ADAFRUIT_SENSORCAL
#include <Adafruit_Sensor_Calibration.h>
#endif

#ifdef DRV_TMP117
#include <Adafruit_TMP117.h>
#endif

#ifdef DRV_BMI270_BMM150
#include <Arduino_BMI270_BMM150.h>
#endif

#ifdef SERIAL_GPS
#include <TinyGPS++.h>
extern TinyGPSPlus serialGps;
#endif

#ifdef TREEWALK
#include "TreeWalker.hpp"
#endif

#ifdef DRV_FLOWSENSOR
#include <FlowSensor.hpp>
#endif

#ifdef LITTLEFS
#include <LittleFS.h>
#endif

#ifdef DRV_MPU9250
#include <SparkFunMPU9250-DMP.h>
#endif

#ifdef UBLOX_GPS
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#endif

#ifdef FASTLED
#include <FastLED.h>
#endif

#define B2S(x)  ((x) ? "true" : "false")
#define T2OK(x) ((x) ? "OK" : "FAILED")

#define RAD2DEG(r) ((r)*57.29577951f)
#define DEG2RAD(d) ((d)*0.017453292f)
#define G2MG(d)    ((d)*1000.)

#define TOGGLE(pin) digitalWrite(pin, !digitalRead(pin))

#define STRINGIFY(x) #x
#define TOSTRING(x)  STRINGIFY(x)
#define AT           __FILE__ ":" TOSTRING(__LINE__)

#define SSID_SIZE   32
#define IPADR_SIZE  32
#define WSPATH_SIZE 32
#define PATH_SIZE   64
#define NUM_SSID    10
#define NUM_SPI     2

typedef enum {
    DEV_NONE,
    DEV_FXOS8700_FXAS21002C,
    DEV_MPU6886,
    DEV_MPU9250_DMP,
    DEV_MPU6050,
    DEV_BMI270_BMM150,
    DEV_ICM20948,
    DEV_BNO08x,
    DEV_M5UNIFIED,
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
    ACCEL_ICM20948,
    ACCEL_M5IMU,

} accel_type_t;

typedef enum {
    GYRO_NONE = 0,
    GYRO_FXAS21002C,
    GYRO_MPU6886,
    GYRO_MPU9250_DMP,
    GYRO_MPU6050,
    // GYRO_SH200Q,
    GYRO_BMI270,
    GYRO_ICM20948,
    GYRO_M5IMU,
} gyro_type_t;

typedef enum {
    MAG_NONE = 0,
    MAG_FXOS8700,
    MAG_BMM150,
    MAG_MPU9250_DMP,
    MAG_ICM20948,
    MAG_M5IMU,
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

typedef enum {
    FMT_NDJSON = 1,
    FMT_JSON,
    FMT_PRETTY_JSON,
    FMT_CSV,
    FMT_TELEPLOT
} logformat_t;

typedef struct {
    uint32_t accel_time;
    uint32_t gyro_time;
    uint32_t magnetometer_time;

    uint32_t runtime;           // per IMU read + AHRS update, micros
    uint32_t last_imu_update;   // micros()
    uint32_t last_baro_update;  // micros()

    sensors_event_t accel, gyro, mag;  // as from drivers, possibly calibrated

    // after optionl North-East-Down translation
    sensors_event_t final_accel, final_gyro, final_mag;
    sensors_event_t temp;
    float roll, pitch, heading;
    float qw, qx, qy, qz;
    float gravx, gravy, gravz;
} sensor_state_t;

typedef struct {
    float hpa;  // hectoPascal
    float alt;  // meters
} baro_report_t;

typedef struct {
    float shuntvoltage;
    float busvoltage;
    float current_mA;
    float loadvoltage;
    float power_mW;
} ina219_report_t;

typedef struct {
    float temperature;
} tmp117_report_t;

// example how to extend the slowSensorReport type
typedef struct {
    float milli_rem;
} geiger_report_t;

typedef enum {
    TYPE_NONE = 0,
    TYPE_LPS22,
    TYPE_DPS3XX,
    TYPE_BMP3XX,
    TYPE_INA219,
    TYPE_TMP117,
    TYPE_GEIGER
} slow_sensor_type_t;

typedef struct {
    slow_sensor_type_t typus;
    int64_t timestamp;  
    union {
        baro_report_t baro;
        ina219_report_t ina219;
        tmp117_report_t tmp117;
        geiger_report_t geiger;
    };
} slowSensorReport_t;

typedef struct {
    accel_type_t accel_type;
    const char *accel_name;

    gyro_type_t gyro_type;
    const char *gyro_name;

    mag_type_t magnetometer_type;
    const char *magnetometer_name;

    Adafruit_AHRS_FusionInterface *filter;

    const i2c_probe_t *dev[I2C_MAX];
    bool i2c_avail[NUM_I2C];
    bool serialgps_avail;
    bool flowsensor_avail;

    // bool wire_avail, wire1_avail;

    bool m5_imu_avail;

    IPAddress tpHost;
    int tpPort;
    struct tm timeinfo;      // from NTP query time
    int64_t timeSinceEpoch;  // epoch milliseconds at NTP query time
    unsigned long
        timeinfo_millis;  // mS elapsed since boot when NTP time was measured
    int64_t millis_offset; // add this to millis() to get a abs timestamp

    Adafruit_Sensor_Calibration_EEPROM cal;
    bool cal_loaded;
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

    // WiFi
    bool wifi_avail;
    bool internet_avail;
    bool ntp_time_set;
    bool log_open;
    char log_path[PATH_SIZE];
    SdBaseFile log_fd;

    // SD card etc
    volatile int numCdInterrupts;
    volatile bool cdLastState;
    volatile uint32_t cdDebounceTimeout;
    // portMUX_TYPE mux;
    uint32_t saveCdDebounceTimeout;
    bool saveLastState;
    int save;
    bool sd_mounted, sdfat_healthy;

    // LittleFS
    bool partition_mounted, lfs_healthy;

    // boot counter for filenames etc
    // see
    // https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/examples/StartCounter/StartCounter.ino
    uint32_t boot_count;

    SpiRamJsonDocument *root, *client_result;

    uint32_t ble_ads;

} config_t;  // FIXME should be status_t

extern config_t config;

typedef struct {
    int16_t sda;
    int16_t scl;
    uint16_t kHz;
} i2c_cfg_t;

typedef struct {
    int16_t miso, mosi, sck;
    uint16_t kHz;
} spi_cfg_t;

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
    float ble_rate;
    float sensor_rate;
    device_type_t selected_imu;
    const char *selected_imu_name;
    i2c_dev_t which_baro;
    uint32_t which_kfmask;  // FIXME used?

    // M5 IMU setup params
    uint8_t rotation;  // 0,1,2

    int flowsensor_pin;

    // smoothing
    float alpha;

    // WiFi
    char ssids[NUM_SSID][SSID_SIZE];
    char passwords[NUM_SSID][SSID_SIZE];
    int8_t num_ssid;
    char hostname[IPADR_SIZE];
    char ap_ssid[SSID_SIZE];
    char ap_password[SSID_SIZE];
    // ntp
    char ntp_poolname[IPADR_SIZE];

    // teleplot
    int32_t tpPort;
    char tpHost[SSID_SIZE];

    // serial GPS
    int32_t gps_uart;
    int32_t gps_speed;
    int32_t gps_rx_pin, gps_tx_pin;

    // spdlog
    int log_level;

    // trace flag bitmask
    uint32_t trace;

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
    uint32_t ble_stack;
    int sensor_core;
    int reporter_core;
    int ble_core;
    uint32_t sensor_prio;
    uint32_t reporter_prio;
    uint32_t ble_prio;

    // SD card
    uint32_t sd_wait_ms;
    int16_t sd_cs_pin;
    int16_t sd_card_detect_pin;
    uint32_t sd_freq_kHz;
    char sd_mountpoint[WSPATH_SIZE];
    uint8_t sd_maxfiles;
    bool sd_format_if_empty;

    // SPI
    spi_cfg_t spi_cfg[NUM_SPI];

    // I2C
    i2c_cfg_t i2c_cfg[NUM_I2C];

    // logging
    logformat_t log_format;
    float log_commit_freq;
    bool log_to_sd;

    // LittleFS
    bool lfs_format_if_empty;
    uint8_t lfs_maxfiles;
    char lfs_mountpoint[WSPATH_SIZE];
    char lfs_partition_label[WSPATH_SIZE];

    // PSRAM Cache for webserver
    uint32_t max_psram_for_cache_pct;

    // watchdog
    float watchdog;

  // hilmar extensions
  float hilmar_rate;

} options_t;
extern options_t options;

#if defined(BLE_DECODER)
void setupBLE(options_t &opt, config_t &config);
void BLEscanOnce(options_t &opt, config_t &config);
#endif

// shell
extern CmdCallback<NUM_COMMANDS> cmdCallback;
extern CmdBuffer<CMD_BUFSIZE> buffer;
extern CmdParser shell;

extern volatile bool run_reporter, run_stats, run_sensors;
extern const char *sensor_types[];

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
extern const char *mA;
extern const char *volt;
extern const char *counter_;
extern const char *percent_;

// Streams
extern Fmt Console, serialConsole;
#if defined(TELEPLOT)
extern Teleplot teleplot;
#endif
extern WriteBufferingStream bufferedLogger;
extern Ticker ubloxStartupTicker, stats_ticker;
void flushBuffers(void);

void initShell(void);
void testSerial(void);
void testWebSerial(void);
void i2cScan(void);

#ifdef UBLOX_GPS
void ublox_setup(void);
bool ublox_detect(options_t &opt, config_t &config);
void ublox_loopcheck(void);
void ublox_nav_pvt(UBX_NAV_PVT_data_t *ub);
#endif
#ifdef SERIAL_GPS
#include "tinygpsmisc.hpp"
#endif

bool readPrefs(options_t &opt);
bool savePrefs(options_t &opt);
void getDefaultPrefs(options_t &opt);
bool wipePrefs(void);
uint32_t readBootCount(void);
uint32_t readWdtCount(void);
void updatedWdtCount(void);

bool selectAHRS(options_t &opt, config_t &config);
// void handleSensors(config_t &config, const options_t &options);
void handleSensors(void);
void reporter(config_t &config, options_t &opt);
void printSensorsDetected(options_t &options, config_t &config);
void printHelp(options_t &options, config_t &config);
void printCurrentCalibration(options_t &opt, config_t &config);
const char *boardType(void);

const char *algoName(ahrs_algo_t algo);

#ifdef MOTIONCAL
void MotionCal(sensor_state_t &state, const options_t &options,
               config_t &config);
extern bool motion_cal;
#endif

bool WifiSetup(const options_t &opt, config_t &config);
int browseService(const char *service, const char *proto);
void printLocalTime(sensor_state_t *state);
void flushAll(void);
void setStatsRate(const float hz);

inline void flagSetter(volatile bool *flag) { *flag = true; }

void setRate(Ticker &ticker, const float Hz, volatile bool *flag);

bool initIMU(options_t &options, config_t &config);
void initOtherSensors(options_t &options, config_t &config);

// webserver
bool configureWebServer(const options_t &opt, config_t &config);

// spdlog
void setup_syslog(void);
void set_syslog_loglevel(const int level);
// tasks

// from:
// https://github.com/yash-sanghvi/ESP32/blob/master/WatchDogTimer/WatchDogTimer.ino
void watchDogRefresh(void);
void watchDogSetup(const options_t &opt);

// timingpins.cpp:
void init_timingpins(void);
int64_t abs_timestamp(const config_t &config);

#define OPT(x) options->x

extern Adafruit_FXOS8700 *fxos;
extern Adafruit_FXAS21002C *fxas;

extern Adafruit_Sensor *accelerometer;
extern Adafruit_Sensor *magnetometer;
extern Adafruit_Sensor *gyroscope;

extern Adafruit_BMP3XX *bmp;
extern Adafruit_LPS22 *lps;
extern Adafruit_DPS310 *dps3xx;
extern Adafruit_INA219 *ina219;
extern Adafruit_TMP117 *tmp117;

extern Adafruit_Sensor *lps2x_pressure;
extern Adafruit_Sensor *dps3xx_pressure;
extern BoschSensorClass *bmi270_bmm150;
extern Adafruit_ICM20948 *icm20948;
#if defined(DRV_BNO08x)
extern Adafruit_BNO08x *bno08x
#endif
    extern Adafruit_MPU6050 *mpu6050;
extern MPU9250_DMP *mpu9250_dmp;
extern Adafruit_MPU6886 *mpu6886;
extern FlowSensor flow_sensor;
