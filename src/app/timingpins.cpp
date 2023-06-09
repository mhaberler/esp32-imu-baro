#include "defs.hpp"

void init_timingpins(void) {
#ifdef IMU_PIN
    pinMode(IMU_PIN, OUTPUT);
    digitalWrite(IMU_PIN, LOW);
#endif
#ifdef REPORT_PIN
    pinMode(REPORT_PIN, OUTPUT);
    digitalWrite(REPORT_PIN, LOW);
#endif
#ifdef EXTRA_PIN
    pinMode(EXTRA_PIN, OUTPUT);
    digitalWrite(EXTRA_PIN, LOW);
#endif
#ifdef BACKGROUND_PIN
    pinMode(BACKGROUND_PIN, OUTPUT);
    digitalWrite(BACKGROUND_PIN, LOW);
#endif
#ifdef ISR_PIN
    pinMode(ISR_PIN, OUTPUT);
    digitalWrite(ISR_PIN, LOW);
#endif
#ifdef UBLOX_PIN
    pinMode(UBLOX_PIN, OUTPUT);
    digitalWrite(UBLOX_PIN, LOW);
#endif
#ifdef ASYNC_TCP_PIN
    pinMode(ASYNC_TCP_PIN, OUTPUT);
#endif
#ifdef TRACE1_PIN
    pinMode(TRACE1_PIN, OUTPUT);
#endif
#ifdef TRACE2_PIN
    pinMode(TRACE2_PIN, OUTPUT);
#endif
#ifdef TRACE3_PIN
    pinMode(TRACE3_PIN, OUTPUT);
#endif
#ifdef TRACE4_PIN
    pinMode(TRACE4_PIN, OUTPUT);
#endif

    
}
