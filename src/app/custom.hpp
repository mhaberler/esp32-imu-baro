#ifndef __CUSTOMVALUES_H__
#define __CUSTOM_H__
#include "defs.hpp"

// called during setup()
void customInitCode(const config_t &config, options_t &options);

// called at the end of the reporting function, once all sensors have been read
void customReportingRateCode(const sensor_state_t &state,
                             const options_t &options);

// called after all IMU processing is done and imu_state is current
void customIMUrateCode(const sensor_state_t &state, const options_t &options);

// called at the end of help()
void customHelpText(const config_t &config, const options_t &options);

// called at the end of initShell()
void customCommands(const config_t &config, options_t &options);
#endif