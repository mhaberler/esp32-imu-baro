
#include "defs.hpp"

hw_timer_t *watchDogTimer = NULL;

static bool custom_wdt_enabled = false;

void IRAM_ATTR watchDogInterrupt() {
    Serial.println("custom watchdog reset");
    updatedWdtCount();
    ESP.restart();
}

void watchDogRefresh() {
    if (custom_wdt_enabled)
        timerWrite(watchDogTimer, 0);  // reset timer (feed watchdog)
}

void watchDogSetup(const options_t &opt) {
    if (opt.watchdog > 0) {
        LOGD("watchDogSetup: {} sec", opt.watchdog);
#if 0
    watchDogTimer = timerBegin(2, 80, true);
    timerAttachInterrupt(watchDogTimer, &watchDogInterrupt, true);
    timerAlarmWrite(watchDogTimer, opt.watchdog * 1000000, false);
    timerAlarmEnable(watchDogTimer);
#else
        LOGE("------- watchDogSetup MISSING IMPLEMENTATION");

#endif
        custom_wdt_enabled = true;
    } else {
        LOGD("watchDogSetup: DISABLED");
    }
}
