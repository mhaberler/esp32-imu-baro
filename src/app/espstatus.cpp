#include "espstatus.hpp"
#include "Esp.h"
#include "buildinfo.h"
#include <Fmt.h>

void heap_report(Fmt &s, const char *fn, const int line) {

  if (fn && line) {
    s.fmt("task report from: {}:{}\n", fn, line);
  }
  s.fmt("task {} core {} stack high water mark {}\n", pcTaskGetTaskName(NULL),
        xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL));

  s.fmt("Total heap: {}\n", ESP.getHeapSize());
  s.fmt("Free heap: {}\n", ESP.getFreeHeap());
  s.fmt("lowest level of free heap since boot: {}\n", ESP.getMinFreeHeap());
  s.fmt("largest block of heap that can be allocated at once: {}\n",
        ESP.getMaxAllocHeap());
}

void psram_report(Fmt &s, const char *fn, const int line) {
  if (fn && line) {
    s.fmt("psram_report from: {}:{}\n", fn, line);
  }
  s.fmt("Total PSRAM: {}\n", ESP.getPsramSize());
  s.fmt("Used PSRAM: {}\n", ESP.getPsramSize() - ESP.getFreePsram());
  s.fmt("Free PSRAM: {}\n", ESP.getFreePsram());
  s.fmt("MinFreePsram: {}\n", ESP.getMinFreePsram());
  s.fmt("getMaxAllocPsram: {}\n", ESP.getMaxAllocPsram());
}

const char *flashChipSpeed(void) {
  uint8_t byte = ESP.getFlashChipSpeed();
  switch (byte & 0x0F) {
  case 0x0: // 40 MHz
    return "40 MHz";
  case 0x1: // 26 MHz
    return "26 MHz";
  case 0x2: // 20 MHz
    return "20 MHz";
  case 0xf: // 80 MHz
    return "80 MHz";
  default: // fail?
    return "???";
  }
}

const char *flashChipSize(void) {
  uint8_t byte = ESP.getFlashChipSize();
  switch (byte & 0x0F) {
  case 0x0: // 8 MBit (1MB)
    return "1 MB";
  case 0x1: // 16 MBit (2MB)
    return "2 MB";
  case 0x2: // 32 MBit (4MB)
    return "4 MB";
  case 0x3: // 64 MBit (8MB)
    return "8 MB";
  case 0x4: // 128 MBit (16MB)
    return "16 MB";
  default: // fail?
    return "????";
  }
}

void platform_report(Fmt &s) {
  s.fmt("model: {} rev: {} cores: {} cpufreq: {} sdkversion: {}\n",
        ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(),
        ESP.getCpuFreqMHz(), ESP.getSdkVersion());
  s.fmt("PS RAM size: {} PS RAM free: {}\n", ESP.getPsramSize(),
        ESP.getFreePsram());
  s.fmt("Flash size: {} speed: {}\n", flashChipSize(), flashChipSpeed());
}

void build_setup_report(Fmt &s) {

  s.fmt("git SHA: {}\n", GIT_REV);
  s.fmt("git branch: {}\n", GIT_BRANCH);
  s.fmt("Author: {}\n", GIT_AUTHOR);
  s.fmt("Subject: {}\n", GIT_SUBJECT);
  s.fmt("Commit date: {}\n", GIT_COMMIT_DATE);

  s.fmt("compiled " __DATE__ " " __TIME__ " \n");
  s.fmt("PLATFORMIO={}\n", PLATFORMIO);
  s.fmt("ARDUINO_BOARD={}\n", ARDUINO_BOARD);
  s.fmt("ARDUINO_VARIANT={}\n", ARDUINO_VARIANT);
}