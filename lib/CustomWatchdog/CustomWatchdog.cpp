
#include "CustomWatchdog.hpp"

#if CUSTOM_WATCHDOG_SECONDS > 0
hw_timer_t *watchDogTimer = NULL;

void IRAM_ATTR watchDogInterrupt() {
  Serial.println("custom watchdog reset - reboot");
  ESP.restart();
}

void watchDogRefresh() {
  timerWrite(watchDogTimer, 0); // reset timer (feed watchdog)
}

void watchDogSetup(const uint32_t seconds) {
  watchDogTimer = timerBegin(2, 80, true);
  timerAttachInterrupt(watchDogTimer, &watchDogInterrupt, true);
  timerAlarmWrite(watchDogTimer, CUSTOM_WATCHDOG_SECONDS * 1000000, false);
  timerAlarmEnable(watchDogTimer);
}
#else
void watchDogRefresh() {}
void watchDogSetup(const uint32_t seconds) {}
#endif
