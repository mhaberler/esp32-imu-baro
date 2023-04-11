#ifndef __UTIL_H__
#define __UTIL_H__

#include <Arduino.h>
#define SEALEVELPRESSURE_HPA (1013.25)

float Pascal2meters(float pascal, float seaLevel = SEALEVELPRESSURE_HPA);
#endif