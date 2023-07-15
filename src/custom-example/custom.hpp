#ifndef __CUSTOMVALUES_H__
#define __CUSTOM_H__
#include "../app/defs.hpp"

struct custom_t {
#ifdef SMOOTHING_DEMO
    float smoothedBaroAlt = NAN;
    float alpha;
#endif

    float env_temp_C;
    float env_hum_pct;
    uint32_t env_millis;
    float oat_temp_C;
    float oat_hum_pct;
    uint32_t oat_millis;

    // float flow1_counts;
};

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