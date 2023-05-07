
#pragma once
#include "defaults.hpp"

bool init_serial_gps(const options_t &opt);
void check_serial_gps(void);
void serialGpsStats(void);

#define GPSFIX3D(gps)                                                          \
  ((gps.location.age() < GPS_MAXAGE) && (gps.altitude.age() < GPS_MAXAGE))
#define GPSPTIME_VALID(gps)                                                    \
  ((gps.time.age() < GPS_MAXAGE) && (gps.date.age() < GPS_MAXAGE))
