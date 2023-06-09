#include "SdFat.h"
#include "SdFatConfig.h"

SdBaseFile _file;

bool listDir(const char *dir) {
  Serial.printf("files in %s:\n", dir);
  SdBaseFile root;
  if (!root.open("/")) {
    Serial.printf("dir.open(%s) failed\n", dir);
    return false;
  }
  while (_file.openNext(&root, O_RDONLY)) {
    _file.printFileSize(&Serial);
    Serial.write(' ');
    _file.printModifyDateTime(&Serial);
    Serial.write(' ');
    _file.printName(&Serial);
    char f_name[EXFAT_MAX_NAME_LENGTH];
    _file.getName(f_name, EXFAT_MAX_NAME_LENGTH);
    if (_file.isDir()) {
      Serial.write('/');
    }
    Serial.println();
    _file.close();
  }
  if (root.getError()) {
    Serial.println(F("openNext failed"));
    return false;
  }
  return true;
}

