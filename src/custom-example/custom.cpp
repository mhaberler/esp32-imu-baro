#include "custom.hpp"
#include "defaults.hpp"
#include <CmdBuffer.hpp>
#include <CmdCallback.hpp>
#include <CmdParser.hpp>
#include <cmath>

#define DEFAULT_ALPHA 0.05 // weight of last sample wrt history

// none of contents of custom_t is visible or modifiable outside this file.
struct custom_t {
#ifdef SMOOTHING_DEMO
  float smoothedBaroAlt = NAN;
  float alpha;
#endif
};

struct custom_t custom;

void customInitCode(const config_t &config, options_t &options) {

#ifdef SMOOTHING_DEMO
  custom.alpha = DEFAULT_ALPHA;
#endif
  LOGD("customInitCode done");
}

void customIMUrateCode(const sensor_state_t &state, const options_t &options) {
#ifdef CUSTOM_PIN
  TOGGLE(CUSTOM_PIN);
#endif

// custom code here
#ifdef CUSTOM_PIN
  TOGGLE(CUSTOM_PIN);
#endif
}

void customReportingRateCode(const sensor_state_t &state,
                             const options_t &options) {
#ifdef REPORT_PIN
  TOGGLE(REPORT_PIN);
#endif
  if (options.teleplot_viewer) {
#ifdef SMOOTHING_DEMO
    // switch (options.which_baro) {
    // case /* constant-expression */:
    //   /* code */
    //   break;

    // default:
    //   break;
    // }
    // const baro_report_t &bp = state.baro_values[options.which_baro];
    // if (!std::isnan(custom.smoothedBaroAlt)) {
    //   custom.smoothedBaroAlt =
    //       custom.alpha * bp.alt + (1 - custom.alpha) * custom.smoothedBaroAlt;
    //   teleplot.update_ms("smoothedBaroAlt", millis(), custom.smoothedBaroAlt,
    //                      meter, TELEPLOT_FLAG_NOPLOT);
    // } else {
    //   custom.smoothedBaroAlt = bp.alt;
    // }

#endif
  }
#ifdef REPORT_PIN
  TOGGLE(REPORT_PIN);
#endif
}

void customHelpText(const config_t &config, const options_t &options) {
#ifdef SMOOTHING_DEMO
  LOGD("  alpha # set smoothing alpha: {}", options.alpha);
#endif
}

void customCommands(const config_t &config, options_t &options) {

  LOGD("customCommands:");
#ifdef SMOOTHING_DEMO
  cmdCallback.addCmd("ALPHA", [](CmdParser *cp) {
    if (cp->getParamCount() == 1) {
      LOGD("alpha: {}", custom.alpha);
      return;
    }
    custom.alpha = atof(cp->getCmdParam(1));
    LOGD("setting alpha: {}", custom.alpha);
  });
#endif
}
