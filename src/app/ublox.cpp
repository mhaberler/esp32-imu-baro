#include "defs.hpp"

#ifdef UBLOX

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

SFE_UBLOX_GNSS myGNSS;

void ublox_loopcheck(void) {
  myGNSS.checkUblox();
  myGNSS.checkCallbacks();
}
void ublox_setup() {
  myGNSS.setNavigationFrequency(NAV_FREQUENCY);
  myGNSS.setAutoPVTcallbackPtr(&ublox_nav_pvt);
}

bool ublox_detect(const config_t &config, bool debug) {
  bool found;
  if (debug) {
    myGNSS.enableDebugging(Console, false);
  }
  if (config.wire1_avail) {
    found = myGNSS.begin(Wire1, UBLOX_DEFAULT_ADDRESS);
    //  UBLOX_WAIT, false);
    if (found) {
      LOGD("ublox detected on Wire1");
      return myGNSS.isConnected();
    }
  }
  found = myGNSS.begin(Wire);
  if (found) {
    LOGD("ublox detected on Wire");
    return myGNSS.isConnected();
    ;
  }
  LOGD("no ublox detected on any Wire");
  return false;
}

#else
void ublox_setup() {}
bool ublox_detect(const config_t &config, bool debug) { return false; }
void ublox_loopcheck(void) {};
#endif
