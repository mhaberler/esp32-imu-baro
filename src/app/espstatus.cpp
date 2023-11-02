
#include <sdkconfig.h>
#include "freertos-all.h"
#include "Esp.h"
#include "Version.h"
#include "espstatus.hpp"
#include "logmacros.hpp"
#include <Fmt.h>
#include <cstdarg>

#define STR(x)
void
task_details (bool bitch, const char *taskName, TaskHandle_t handle)
{
  TaskHandle_t t = NULL;
  if (taskName) {
    t = xTaskGetHandle(taskName);
  }
  if (t == NULL) {
    // if (bitch)
    //   LOGD("task: handle {} not found", t);
    return;
  }
  if (taskName == NULL) {
    taskName = pcTaskGetName(handle);
  }
  if (taskName == NULL) {
    // if (bitch)
    //   LOGD("task: name {} not found", t);
    return;
  }
  // affinity == '0x7fffffff' - no affinity, else core #
  const char *affinityStr = "";
  switch (xTaskGetAffinity (t))
    {
    case 0:
      affinityStr = "core 0";
      break;
    case 1:
      affinityStr = "core 1";

      break;
    case 0x7fffffff:
      affinityStr = "none";
      break;
    }

#ifdef configUSE_CORE_AFFINITY // not defined :-/
                               // vTaskCoreAffinitySet(0,3);
#endif
  LOGD ("task: {:<16} affinity: {:<6} prio: {} stack HWM: {}", taskName,
        affinityStr, uxTaskPriorityGet (t), uxTaskGetStackHighWaterMark (t));
}

void
task_report (const char *fn, const int line, ...)
{
  // if (fn && line) {
  //   LOGD("task report from: {}:{}", fn, line);
  // }
  // va_list args;
  // va_start(args, line);
  // const char *tn = va_arg(args, const char *);
  // while (tn != NULL) {
  //   task_details(true, tn);
  //   tn = va_arg(args, const char *);
  // }
  // va_end(args);

  // assorted "well-known" tasks
  task_details (false, "sensor");   // this project
  task_details (false, "reporter"); // this project
  task_details (false, "blescan");  // this project
  task_details (false, "mdns");     // this project

  task_details (false, "nimble_host");  // NimBLE
  task_details (false, "btController"); // NimBLE

  task_details (false, "loopTask");        // Arduino
  task_details (false, "wifi");            // Arduino
  task_details (false, "arduino_events");  // Arduino libraries:
  task_details (false, "https_ota_task");  // Arduino
  task_details (false, "async_udp");       // Arduino
  task_details (false, "spp_tx");          // Arduino BluetoothSerial
                                           // Arduino core:
  task_details (false, "msc_disk");        // FirmwareMSC.cpp
  task_details (false, "toneTask");        // Tone.cpp
  task_details (false, "uart_event_task"); // HardwareSerial.cpp
  task_details (false, "rmt_rx_task");     // esp32-hal-rmt.c
  task_details (false, "i2c_slave_task"); // cores//esp32/esp32-hal-i2c-slave.c
  task_details (false, "usbd");           // esp32-hal-tinyusb.c
  task_details (false, "rmt_rx_task");    // esp32-hal-rmt.c

  task_details (false, "async_tcp");   // Async_TCP
  task_details (false, "spk_task");    // M5Unified
  task_details (false, "mic_task");    // M5Unified
  task_details (false, "task_memcpy"); // M5GFX
  task_details (false, "esp_timer");   // esp-idf soft timer

  task_details (false, "main");            // esp-idf misc
  task_details (false, "console_repl");    // esp-idf misc
  task_details (false, "btm_rrm_t");       // esp-idf misc
  task_details (false, "dppT");            // esp-idf misc
  task_details (false, "wpa2T");           // esp-idf misc
  task_details (false, "esp_wpa3_hostap"); // esp-idf misc
  task_details (false, "wpsT");            // esp-idf misc

  // task_details(true, NULL, xTaskGetIdleTaskHandle()); // FreeRTOS idle task
}

void
heap_report (const char *fn, const int line)
{

  if (fn && line)
    {
      LOGD ("heap report from: {}:{}", fn, line);
    }
  LOGD ("Total heap: {}", ESP.getHeapSize ());
  LOGD ("Free heap: {}", ESP.getFreeHeap ());
  LOGD ("lowest level of free heap since boot: {}", ESP.getMinFreeHeap ());
  LOGD ("largest block of heap that can be allocated at once: {}",
        ESP.getMaxAllocHeap ());
}

void
psram_report (const char *fn, const int line)
{
  if (fn && line)
    {
      LOGD ("psram_report from: {}:{}", fn, line);
    }
  LOGD ("Total PSRAM: {}", ESP.getPsramSize ());
  LOGD ("Used PSRAM: {}", ESP.getPsramSize () - ESP.getFreePsram ());
  LOGD ("Free PSRAM: {}", ESP.getFreePsram ());
  LOGD ("MinFreePsram: {}", ESP.getMinFreePsram ());
  LOGD ("getMaxAllocPsram: {}", ESP.getMaxAllocPsram ());
}

const char *
flashChipSpeed (void)
{
  uint8_t byte = ESP.getFlashChipSpeed ();
  switch (byte & 0x0F)
    {
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

const char *
flashChipSize (void)
{
  uint8_t byte = ESP.getFlashChipSize ();
  switch (byte & 0x0F)
    {
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

void
platform_report (void)
{
  LOGD ("model: {} rev: {} cores: {} cpufreq: {} sdkversion: {}",
        ESP.getChipModel (), ESP.getChipRevision (), ESP.getChipCores (),
        ESP.getCpuFreqMHz (), ESP.getSdkVersion ());
  LOGD ("PS RAM size: {} PS RAM free: {}", ESP.getPsramSize (),
        ESP.getFreePsram ());
  // this is wrong:
  // LOGD("Flash size: {} speed: {}", flashChipSize(), flashChipSpeed());
}

void
build_setup_report (void)
{
  LOGD ("build version: {}", VERSION);
  LOGD ("Arduino Version : {}.{}.{}",ESP_ARDUINO_VERSION_MAJOR,ESP_ARDUINO_VERSION_MINOR,ESP_ARDUINO_VERSION_PATCH);
  LOGD ("ESP-IDF Version : {}.{}.{}",ESP_IDF_VERSION_MAJOR,ESP_IDF_VERSION_MINOR,ESP_IDF_VERSION_PATCH);
  LOGD ("git SHA: {}", GIT_REV);
  LOGD ("git branch: {}", GIT_BRANCH);
  LOGD ("Author: {}", GIT_AUTHOR);
  LOGD ("Subject: {}", GIT_SUBJECT);
  LOGD ("Commit date: {}", GIT_COMMIT_DATE);
  LOGD ("compiled " __DATE__ " " __TIME__ " ");

  LOGD ("PIOENV={}", PIOENV);
  LOGD ("PIOPLATFORM={}", PIOPLATFORM);
  LOGD ("PROJECT_DIR={}", PROJECT_DIR);
  LOGD ("BUILD_DIR={}", BUILD_DIR);
  LOGD ("BUILD_TYPE={}", BUILD_TYPE);
  LOGD ("PLATFORMIO={}", PLATFORMIO);
  LOGD ("ARDUINO_BOARD={}", ARDUINO_BOARD);
  LOGD ("ARDUINO_VARIANT={}", ARDUINO_VARIANT);
}
