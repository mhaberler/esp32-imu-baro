#include "defs.hpp"
#include <Crypto.h>
#include <Preferences.h>
#ifdef SD_SUPPORT
#include <SD.h>
#endif
#define NAMESPACE "preferences"
#define OPTKEY "options"
#define OPTKEY_SHA "sha256"

//
// Support for logging binary data as hex
// format flags, any combination of the following:
// {:X} - print in uppercase.
// {:s} - don't separate each byte with space.
// {:p} - don't print the position on each line start.
// {:n} - don't split the output to lines.
// {:a} - show ASCII if :n is not set

//
// Examples:
//
// std::vector<char> v(200, 0x0b);
// logger->info("Some buffer {}", spdlog::to_hex(v));
// char buf[128];
// logger->info("Some buffer {:X}", spdlog::to_hex(std::begin(buf),
// std::end(buf))); logger->info("Some buffer {:X}",
// spdlog::to_hex(std::begin(buf), std::end(buf), 16));

// #define HEX(b, l) (spdlog::to_hex(b, b + l, l))

// see
// https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/examples/StartCounter/StartCounter.ino

static Preferences prefs;

// compiled-in defaults
static options_t defaults = {
    .ahrs_algo = ALGO_MAHONEY,
    .run_filter = AHRS_FILTER,
    .report_raw = REPORT_RAW,
    .report_hpr = REPORT_HPR,
    .report_quat = REPORT_QUAT,
    .report_baro = REPORT_BARO,
    .report_grav = REPORT_GRAV,
    .teleplot_viewer = TELEPLOT_VIEWER,
    .ndjson = NDJSON,
    .ned = NED_TRANSLATION,
    .timing_stats = REPORT_TIMING,
    .memory_usage = REPORT_MEMORY,
    .report_rate = REPORT_RATE,
    .imu_rate = IMU_RATE,
    .stats_rate = STATS_RATE,
    .selected_imu = DEV_NONE,
    .which_baro = USE_BARO_LPS22,
    .flowsensor_pin = FLOWSENSOR_PIN,
    .alpha = ALPHA,
    .num_ssid = 0,
    {.hostname = HOSTNAME},
    {.ntp_poolname = NTP_POOL},
    .tpPort = -1,
    .gps_uart = GPS_UART,
    .gps_speed = SERIAL_GPS_SPEED,
    .gps_rx_pin = SERIAL_GPS_RXPIN,
    .gps_tx_pin = SERIAL_GPS_TXPIN,
    .debug = 0,
    .run_webserver = true,
    .webserver_port = 80,

    .run_webserial = true,
    .run_tpwebsocket = true,
    {.littlefs_static_path = LFS_PATH},
    {.websocket_path = WEBSOCKET_PATH},
    {.consolesocket_path = CONSOLE_PATH},
    {.web_default_path = INDEX_HTML},
    {.web_user = WEB_USER},
    {.web_pass = WEB_PASS},
    .flush_ms = FLUSH_PERIOD,
    .reporter_stack = REPORTERTASK_STACKSIZE,
    .sensor_stack = SENSORTASK_STACKSIZE,
    .sensor_core = SENSORTASK_CORE,
    .reporter_core = REPORTERTASK_CORE,
    .sensor_prio = SENSORTASK_PRIORITY,
    .reporter_prio = REPORTERTASK_PRIORITY,

#ifdef SD_SUPPORT
#ifdef M5UNIFIED
    .sd_wait_ms = SD_WAIT_MS,
    .sd_cs_pin = GPIO_NUM_4,
    .sd_freq = 4000000,
    {.sd_mountpoint = SD_MOUNTPOINT},
    .sd_maxfiles = MAX_OPEN_FILES,
    .sd_format_if_empty = false,
#else
    .sd_cs_pin = -1, // disables SD
#endif
#endif
#ifdef LITTLEFS_SUPPORT
    .lfs_format_if_empty = false,
    .lfs_maxfiles = MAX_OPEN_FILES,
    {.lfs_mountpoint = LFS_MOUNTPOINT},
    {.lfs_partition_label = LFS_PARTITION},
#endif
};

bool readPrefs(options_t &opt) {

  byte saved_chksum[SHA256_SIZE];
  memset(saved_chksum, 0, sizeof(saved_chksum));

  prefs.begin(NAMESPACE, true); // R/O
  size_t sha_len =
      prefs.getBytes(OPTKEY_SHA, &saved_chksum, sizeof(saved_chksum));

  size_t opt_len = prefs.getBytes(OPTKEY, (void *)&opt, sizeof(opt));
  prefs.end();

  if (sha_len != SHA256_SIZE) {
    LOGE("readPrefs: checksum read failed, len = {}", sha_len);
    return false;
  }
  if (opt_len != sizeof(opt)) {
    LOGE("readPrefs: options read failed, expected {}, got {}", sizeof(opt),
         opt_len);
    return false;
  }

  SHA256 cksum;
  cksum.doUpdate((const byte *)&opt, opt_len);

  if (!cksum.matches(saved_chksum)) {
    byte expected[SHA256_SIZE];
    cksum.doFinal(expected);
    LOGE("readPrefs: checksum mismatch");
    std::string e((char *)expected, SHA256_SIZE);
    std::string g((char *)saved_chksum, SHA256_SIZE);
    LOGE("expected {}", spdlog::to_hex(e));
    LOGE("got {}", spdlog::to_hex(g));
    return false;
  }
  LOGI("readPrefs: configuration is {}",
       bcmp((const void *)&opt, (const void *)&defaults, sizeof(opt))
           ? "modified"
           : "default");

  return true;
}

void getDefaultPrefs(options_t &opt) { opt = defaults; }

bool savePrefs(options_t &opt) {
  SHA256 cksum;
  byte new_chksum[SHA256_SIZE];

  cksum.doUpdate((const byte *)&opt, sizeof opt);
  cksum.doFinal(new_chksum);

  prefs.begin(NAMESPACE, false); // R/W
  size_t len = prefs.putBytes(OPTKEY, (void *)&opt, sizeof(opt));
  size_t hlen = prefs.putBytes(OPTKEY_SHA, new_chksum, sizeof(new_chksum));
  prefs.end();
  LOGD("saving options: {} chksum: {}", T2OK(len == sizeof(opt)),
       T2OK(hlen != SHA256_SIZE));
  return ((len == sizeof(options_t)) && (hlen == SHA256_SIZE));
}

bool wipePrefs(void) {
  prefs.begin(NAMESPACE, false); // R/W
  prefs.clear();
  prefs.end();
  return true;
}
