#include "defs.hpp"

#include <TinyGPS++.h>

TinyGPSPlus serialGps;
HardwareSerial GPSSerial(GPS_UART);

bool init_serial_gps(const options_t &opt) {

  GPSSerial.begin(opt.gps_speed, SERIAL_8N1, opt.gps_rx_pin, opt.gps_tx_pin);
  LOGD("serialGPS enabled on UART{}", GPS_UART);
  return true;
}

void check_serial_gps(void) {
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    serialGps.encode(c);
  }
}

void serialGpsStats(void) {

  LOGD("charsProcessed: {}", serialGps.charsProcessed());
  LOGD("sentencesWithFix: {}", serialGps.sentencesWithFix());
  LOGD("failedChecksum: {}", serialGps.failedChecksum());
  LOGD("passedChecksum: {}", serialGps.passedChecksum());
}
