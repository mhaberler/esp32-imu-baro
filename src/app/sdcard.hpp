#pragma once

#include "defs.hpp"

void describeCard(SdFat &sd);
bool handleSD(const bool cardPresent, SdFat &sdfat, options_t &opt);
bool init_sd(options_t &opt, config_t &config);
bool sd_openlog(const options_t &opt, config_t &config);
bool listDir(const char *dir);

extern SdFat sdfat;
extern SdBaseFile file;

