#include "defs.hpp"
#include "probe.hpp"

#ifdef UBLOX_GPS

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

bool ublox_detect(options_t &opt, config_t &config) {
  if (opt.trace  & INFO_UBLOX) {
    myGNSS.enableDebugging(Console,  (opt.trace  & INFO_UBLOX_ALL));
  }
  const i2c_probe_t **dev = &config.dev[I2C_UBLOXI2C];
  *dev = probe_dev("ubloxi2c", ublox_devs);
  if (*dev) {
    return true;
  }
  return false;
}

#else
void ublox_setup() {}
bool ublox_detect(const config_t &config, bool debug) { return false; }
void ublox_loopcheck(void){};
#endif
