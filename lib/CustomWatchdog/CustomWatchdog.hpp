#pragma once
#include <Arduino.h>

// from:
// https://github.com/yash-sanghvi/ESP32/blob/master/WatchDogTimer/WatchDogTimer.ino

// activate by defining -DCUSTOM_WATCHDOG
void watchDogRefresh(void);
void watchDogSetup(const uint32_t seconds = 3);
