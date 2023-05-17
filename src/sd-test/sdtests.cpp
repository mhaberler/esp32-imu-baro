#include "SD.h"
#include "SPI.h"
#include "FS.h"
#include "SdFat.h"

SdFat sd;

SdFile file;


#ifdef WROVER_KIT
  if (!sd.begin()) {
    Serial.println("begin failed");
    return;
  }
  file.open("SizeTest.txt", O_RDWR | O_CREAT | O_AT_END);

  file.println("Hello");

  file.close();
  Serial.println("Done");

// // https://github.com/espressif/arduino-esp32/issues/1131#issuecomment-613174510
//   SPI.begin(14, 2, 15, 13);
//   if (!SD.begin(13)) {
//     LOGD("SD  total {}, used {} ({:.1f}%)", SD.totalBytes(),
//                 SD.usedBytes(), 100.0 * SD.usedBytes() / SD.totalBytes());
//     // listDir(SD, "/www", 2);
//   } else {
//     LOGD("SD init failed\n");
//   }
#endif