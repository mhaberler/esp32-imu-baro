

#include "sdcard.hpp"

SdFat sdfat;
SdBaseFile file;

void IRAM_ATTR cardInsertISR(void) {
    // portENTER_CRITICAL_ISR(&mux);
    // numCdInterrupts++;
    // cdLastState = digitalRead(CARD_DETECT_PIN);
    // cdDebounceTimeout =
    //     xTaskGetTickCount(); // version of millis() that works from interrupt
    // portEXIT_CRITICAL_ISR(&mux);
}

void describeCard(SdFat &sd) {
    LOGD("SdFat version: {}", SD_FAT_VERSION_STR);
    LOGD("card size: {:.0f} GB", sd.card()->sectorCount() * 512E-9);
    switch (sd.fatType()) {
        case FAT_TYPE_EXFAT:
            LOGD("FS type: exFat");
            break;
        case FAT_TYPE_FAT32:
            LOGD("FS type: FAT32");
            break;
        case FAT_TYPE_FAT16:
            LOGD("FS type: FAT16");
            break;
        case FAT_TYPE_FAT12:
            LOGD("FS type: FAT12");
            break;
    }
    cid_t cid;
    if (!sd.card()->readCID(&cid)) {
        LOGE("readCID failed");
        return;
    }
    LOGD("Manufacturer ID: 0x{:x}", int(cid.mid));
    LOGD("OEM ID: 0x{:x} 0x{:x}", cid.oid[0], cid.oid[1]);
    LOGD("Product: '{}'", fmt::string_view(cid.pnm, sizeof(cid.pnm)));
    LOGD("Revision: {}.{}", cid.prvN(), cid.prvM());
    LOGD("Serial number: {}", cid.psn());
    LOGD("Manufacturing date: {}/{}", cid.mdtMonth(), cid.mdtYear());
}

bool handleSD(const bool cardPresent, SdFat &sdfat, options_t &opt) {
    uint32_t start = millis();
    LOGD("handleSD sdCspin={} freq={}", opt.sd_cs_pin, opt.sd_freq_kHz);

    if (cardPresent) {
        while (1) {
            bool success = sdfat.begin(SdSpiConfig(
                opt.sd_cs_pin, SHARED_SPI, SD_SCK_KHZ(opt.sd_freq_kHz)));
            if (success) {
                LOGD("SD mounted.");
                describeCard(sdfat);
                return true;
            }
            delay(500);
            if ((millis() - start) > SD_WAIT_MS) {
                LOGD("giving up on SD.");
                return false;
            }
            LOGD("SD waiting... {}", (millis() - start) / 1000.0);
        }
    } else {
        return false;
    }
}

bool init_sd(options_t &opt, config_t &config) {
    if (opt.sd_cs_pin < 0) {
        LOGI("SD support disabled via config (sd_cs_pin < 0).");
        return false;
    }

    if (options.sd_card_detect_pin > -1) {
        // Adafruit SD card breakout: this pin shorts to ground if NO card is
        // inserted
        pinMode(options.sd_card_detect_pin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(options.sd_card_detect_pin),
                        cardInsertISR, CHANGE);
        LOGD("attaching card detect IRQ on pin {}, current state: {}",
             options.sd_card_detect_pin,
             digitalRead(options.sd_card_detect_pin));
    }
    // if the default SPI has not been initialized (eg by initing a display)
    // do so now
    //
    // for pins, see
    // https://github.com/espressif/esp-idf/blob/master/examples/storage/sd_card/sdspi/README.md#pin-assignments
    // assume SPI 0 for SD
    spi_cfg_t *sc = &options.spi_cfg[0];
    if (sc->miso > -1) {
        SPI.begin(sc->sck, sc->miso, sc->mosi);
    }
    config.sd_mounted = handleSD(true, sdfat, opt);
    if (config.sd_mounted) {
        config.sdfat_healthy = true;

        LOGD("SD mounted, toplevel directory:");
        listDir("/");
#ifdef TREEWALKER
        LOGD("fsVisitor:");
        fsVisitor(sdfat, Console, "/", VA_PRINT);
#endif
    }
    return true;
}

bool sd_openlog(const options_t &opt, config_t &config) {
    if (config.sdfat_healthy) {
        if (!sdfat.exists(LOG_SUBDIR)) {
            sdfat.mkdir(LOG_SUBDIR);
        }
        strcpy(config.log_path,
               fmt::format("{}/log_{:06}.njs", LOG_SUBDIR, config.boot_count)
                   .c_str());

        config.log_fd = sdfat.open(config.log_path, O_WRONLY | O_CREAT);
        LOGD("openend SD: '{}' for logging/w: {}", config.log_path,
             T2OK(config.log_fd.isOpen()));

        return config.log_fd.isOpen();
    }
    return false;
}

bool listDir(const char *dir) {
    Serial.printf("files in %s:\n", dir);
    SdBaseFile root, _file;
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
        Serial.println("openNext failed");
        return false;
    }
    return true;
}
