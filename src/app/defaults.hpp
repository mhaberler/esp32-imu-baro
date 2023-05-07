// -- configurable defaults --

#define AHRS_FILTER 1
#define TELEPLOT_VIEWER 0
#define TELEPLOT_3D 0
#define REPORT_RAW 0
#define REPORT_HPR 0
#define REPORT_QUAT 0
#define REPORT_BARO 0
#define REPORT_GRAV 0
#define NDJSON 1
#define NED_TRANSLATION 0
#define ALPHA 0.8

#define USE_QUATERNIONS // else Euler angles for 3D teleplot

#ifndef HOSTNAME
#define HOSTNAME "esp32-imu-baro"
#endif

#ifndef REPORT_RATE
#define REPORT_RATE 0.0 // < 0: no updates
#endif
#ifndef IMU_RATE
#define IMU_RATE 100.0
#endif

#ifndef STATS_RATE
#define STATS_RATE 0.0
#endif

#ifndef BACKGROUND_RATE
#define BACKGROUND_RATE 10.0
#endif

#ifndef REPORT_MEMORY
#define REPORT_MEMORY false
#endif
#ifndef REPORT_TIMING
#define REPORT_TIMING false
#endif

#ifndef FLOWSENSOR_PIN
#define FLOWSENSOR_PIN -1 // disabled
#endif

#ifndef GPS_MAXAGE
#define GPS_MAXAGE 2000 // millis
#endif

#ifndef GPS_UART
#define GPS_UART 2
#endif
#ifndef SERIAL_GPS_SPEED
#define SERIAL_GPS_SPEED 9600
#endif
#ifndef SERIAL_GPS_RXPIN
#define SERIAL_GPS_RXPIN 13
#endif
#ifndef SERIAL_GPS_TXPIN
#define SERIAL_GPS_TXPIN 14
#endif

#define GYRO_SAMPLES 500 // for calibration

#define BMP3XX_ALTERNATE_ADDRESS (0x76)
#define LPS2X_ALTERNATE_ADDRESS (0x5c)
#define DPS3XX_ALTERNATE_ADDRESS (0x76)
#define CMPS14_I2CADDR_DEFAULT 0x60

#define UBLOX_DEFAULT_ADDRESS (0x42)
#define UBLOX_WAIT 5000
#define NAV_FREQUENCY 10 //  per second

#define JSON_SIZE 4096
#define NUM_COMMANDS 50
#define CMD_BUFSIZE 256


#define APP_CPU 1   // has good timing
#define PRO_CPU 0   // does all the housekeeping

#define SENSORTASK_CORE APP_CPU
#define SENSORTASK_PRIORITY 3
#define SENSORTASK_STACKSIZE 16384

#define REPORTERTASK_CORE PRO_CPU
#define REPORTERTASK_PRIORITY 2 // loop task has 1
#define REPORTERTASK_STACKSIZE 16384

#define BACKGROUNDTASK_CORE PRO_CPU
#define BACKGROUNDTASK_PRIORITY 2
#define BACKGROUNDTASK_STACKSIZE 8192

