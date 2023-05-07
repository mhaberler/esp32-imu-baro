#include "defs.hpp"
#include <Preferences.h>

#define NAMESPACE "bwsensor"
#define OPTKEY "options"

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
    .background_rate = BACKGROUND_RATE,
    .selected_imu = DEV_NONE,
    .which_baro = USE_BARO_LPS22,
    .flowsensor_pin = FLOWSENSOR_PIN,
    .alpha = ALPHA,
    .num_ssid = 0,
    .tpPort = -1,
    .gps_uart = GPS_UART,
    .gps_speed = SERIAL_GPS_SPEED,
    .gps_rx_pin = SERIAL_GPS_RXPIN,
    .gps_tx_pin = SERIAL_GPS_TXPIN,
    .debug = 0,
};

bool readPrefs(options_t &opt) {
  size_t len;

  prefs.begin(NAMESPACE, true); // R/O
  len = prefs.getBytes(OPTKEY, (void *)&opt, sizeof(opt));
  prefs.end();

  if ((len == 0) || (len != sizeof(options_t))) {
    if (len == 0) {
      Console.fmt("preferences not set in NVRAM, setting defaults\n");
    } else {
      Console.fmt("definition of options_t changed - setting defaults (old "
                  "size={}, new size={})\n",
                  len, sizeof(options_t));
    }
    bool rc = savePrefs(defaults);
    Console.fmt("preferences saving: {}\n", rc ? "SUCCESS" : "FAIL");
    // now read defaults:
    prefs.begin(NAMESPACE, true); // R/O
    len = prefs.getBytes(OPTKEY, (void *)&opt, sizeof(opt));
    prefs.end();
    if (len == sizeof(opt)) {
      Console.fmt("preferences; successfully read back defaults\n");
      return true;
    }
    Console.fmt("preferences; reading back defaults FAIL\n");
    return false;
  }

  Console.fmt("preferences read vom NVRAM\n");
  return true;
}

bool savePrefs(options_t &opt) {
  prefs.begin(NAMESPACE, false); // R/W
  size_t len = prefs.putBytes(OPTKEY, (void *)&opt, sizeof(opt));
  prefs.end();

  Console.fmt("saving setting permanently: {}\n",
              (len == 0) ? "FAIL" : "SUCCESS");
  return (len == sizeof(options_t));
}

bool wipePrefs(void) {
  prefs.begin(NAMESPACE, false); // R/W
  prefs.clear();
  prefs.end();
  return true;
}
