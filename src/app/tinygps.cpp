#include "defs.hpp"

#include <TinyGPS++.h>

TinyGPSPlus serialGps;
HardwareSerial GPSSerial(GPS_UART);

bool init_serial_gps(const options_t &opt) {

  GPSSerial.begin(opt.gps_speed, SERIAL_8N1, opt.gps_rx_pin, opt.gps_tx_pin);
  Console.fmt("serialGPS enabled on UART{}\n", GPS_UART);
  return true;
}

void check_serial_gps(void) {
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    serialGps.encode(c);
  }
}

void serialGpsStats(void) {

  Console.fmt("charsProcessed: {}\n", serialGps.charsProcessed());
  Console.fmt("sentencesWithFix: {}\n", serialGps.sentencesWithFix());
  Console.fmt("failedChecksum: {}\n", serialGps.failedChecksum());
  Console.fmt("passedChecksum: {}\n", serialGps.passedChecksum());
}
