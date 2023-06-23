#pragma once

   
#if defined(PLATFORM_M5CORE2)

#define I2C0_SDA 21
#define I2C0_SCL 22
#define I2C0_KHZ 400

#define I2C1_SDA 32
#define I2C1_SCL 33
#define I2C1_KHZ 100

#define SD_CS_PIN 4
#define SD_CARD_DETECT_PIN -1 
#define SD_SPI_FREQ     25000

#endif

#if defined(PLATFORM_M5CORE2_BOTTOM2)
#define I2C0_SDA 21
#define I2C0_SCL 22
#define I2C0_KHZ 400

#define I2C1_SDA 32
#define I2C1_SCL 33
#define I2C1_KHZ 100

#define SD_CS_PIN 4
#define SD_CARD_DETECT_PIN -1 
#define SD_SPI_FREQ     25000

#define FLOWSENSOR_PIN 36
#endif

#if defined(PLATFORM_M5CORES3)
#define I2C0_SDA 12
#define I2C0_SCL 11
#define I2C0_KHZ 100

#define I2C1_SDA 2
#define I2C1_SCL 1
#define I2C1_KHZ 100

#define FLOWSENSOR_PIN 36

#define SPI0_MISO -1
#define SPI0_MOSI -1
#define SPI0_SCK -1

#define SPI1_MISO -1
#define SPI1_MOSI -1
#define SPI1_SCK -1

#define SD_CS_PIN          4
#define SD_CARD_DETECT_PIN -1
#define SD_SPI_FREQ        25000

#endif

#if defined(PLATFORM_M5STAMPS3)
#endif

#if defined(PLATFORM_M5STICKC)
#endif

#if defined(PLATFORM_M5STICKC_PLUS)
#endif

#if defined(PLATFORM_LILYGO_TS3)
#define I2C0_SDA 43
#define I2C0_SCL 44
#define I2C0_KHZ 100
#endif

// ESP32-S3-DEV-KIT-N8R8
// https://www.waveshare.com/wiki/ESP32-S3-DEV-KIT-N8R8
// mah breakout board (8MB psram)
#if defined(PLATFORM_S3DEVKIT_N8R8)

#define I2C0_SDA 2
#define I2C0_SCL 1
#define I2C0_KHZ 400

#define I2C1_SDA 41
#define I2C1_SCL 40
#define I2C1_KHZ 400

#define SPI0_MISO 37
#define SPI0_MOSI 35
#define SPI0_SCK  36

#define SPI1_MISO -1
#define SPI1_MOSI -1
#define SPI1_SCK  -1

#define SD_CS_PIN          34
#define SD_CARD_DETECT_PIN 26
#define SD_SPI_FREQ        25000

// #define SERIAL_GPS_UART  2
#define SERIAL_GPS_RXPIN 17
#define SERIAL_GPS_TXPIN 18
#define SERIAL_GPS_SPEED 9600

#endif

// ESP32-S3-DevKitM-1-N8
// mah breakout board (no psram, conrad.at)
#if defined(PLATFORM_S3DEVKIT_M1N8)

#define I2C0_SDA 2
#define I2C0_SCL 1
#define I2C0_KHZ 400

#define I2C1_SDA 41
#define I2C1_SCL 40
#define I2C1_KHZ 400

#define SPI0_MISO 37
#define SPI0_MOSI 35
#define SPI0_SCK  36

#define SPI1_MISO -1
#define SPI1_MOSI -1
#define SPI1_SCK  -1

#define SD_CS_PIN 34
#define SD_CARD_DETECT_PIN 26 
#define SD_SPI_FREQ     25000

// #define SERIAL_GPS_UART  2
#define SERIAL_GPS_RXPIN 17
#define SERIAL_GPS_TXPIN 18
#define SERIAL_GPS_SPEED 9600

#endif

// Sunton ESP32-S3 7" 800*480 TN Display with Touch
// https://www.makerfabs.com/sunton-esp32-s3-7-inch-tn-display-with-touch.html
#if defined(PLATFORM_SUNTON_S3_TFTOUCH)

#endif

#if defined(PLATFORM_WROVER)

#define FLOWSENSOR_PIN 34

#endif