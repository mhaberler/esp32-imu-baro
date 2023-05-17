#pragma once
#include "defs.hpp"
#include "ArduinoJsonCustom.hpp"


#define CLIENT_MESSAGESIZE 4096
#define WEBSOCKETS_SERVER_CLIENT_MAX 10
#define DEFAULT_CLIENT_INTERVAL 2000

// #define BIT(x) (1UL << (x))

#define INFO_BATTERY BIT(0)
#define INFO_GPS_LOCATION BIT(1)
#define INFO_GPS_STATUS BIT(2)
#define INFO_PRESSURE BIT(3)
#define INFO_TEMPERATURE BIT(4)
#define INFO_WIFI BIT(5)
#define INFO_SENSOR_STATS BIT(6)
#define INFO_MEMORY_STATS BIT(7)
#define INFO_SYSTEM BIT(8)
#define INFO_BUILD BIT(9)
#define INFO_VPROFILE BIT(10)
#define INFO_WINDSALOFT BIT(11)
#define INFO_POINTFORECAST BIT(12)

#define DEFAULT_SUBSCRIPTION (INFO_GPS_LOCATION | INFO_GPS_STATUS)

typedef struct {
  SpiRamJsonDocument *clientMessage;
  int32_t subscription;
  int32_t command;
  uint32_t interval;
  uint32_t last_update;
} client_t;

