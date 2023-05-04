#include "defs.hpp"

#include <TinyGPS++.h>

TinyGPSPlus serialGps;
HardwareSerial GPSSerial(2);

void init_serial_gps(const options_t &opt) {
  GPSSerial.begin(opt.gps_speed, SERIAL_8N1, opt.gps_rx_pin, opt.gps_tx_pin);
  Console.fmt("serial GPS: rx={} tx={} speed={} TinyGPSPlus={}\n",
              opt.gps_rx_pin, opt.gps_tx_pin, opt.gps_speed,
              serialGps.libraryVersion());
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
