
#pragma once

#define SEALEVELPRESSURE_HPA (1013.25)

float hPa2meters(float pascal, float seaLevel = SEALEVELPRESSURE_HPA);