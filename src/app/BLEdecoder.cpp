#ifdef BLE_DECODER

#include "defs.hpp"

#include "NimBLEDevice.h"
#include "decoder.h"

#include "custom.hpp"
extern struct custom_t custom;

NimBLEScan *pBLEScan;

TheengsDecoder decoder;

static StaticJsonDocument<512> doc;

class ScanCallbacks : public NimBLEScanCallbacks {
   public:
    ScanCallbacks(options_t *opt, config_t *config)
        : NimBLEScanCallbacks(), _opt(opt), _config(config) {
    }

    std::string convertServiceData(std::string deviceServiceData) {
        int serviceDataLength = (int)deviceServiceData.length();
        char spr[2 * serviceDataLength + 1];
        for (int i = 0; i < serviceDataLength; i++)
            sprintf(spr + 2 * i, "%.2x", (unsigned char)deviceServiceData[i]);
        spr[2 * serviceDataLength] = 0;
        return spr;
    }

    const BLEsensor_t *findSensor(NimBLEAddress const &mac_adress) {
        for (auto i = 0; i < NUM_BLESENSORS; i++) {
            if (mac_adress == _config->nimble_adr[i]) {
                // Console.fmtln("findSensor: found  {}",
                // mac_adress.toString());
                return &_opt->blesensors[i];
            }
        }
        return NULL;
    }

    void onResult(BLEAdvertisedDevice *advertisedDevice) {
#ifdef TRACE2_PIN
        digitalWrite(TRACE2_PIN, HIGH);
#endif
        _config->ble_ads += 1;

        JsonObject BLEdata = doc.to<JsonObject>();

        const BLEsensor_t *bp = findSensor(advertisedDevice->getAddress());
        if (bp == NULL) {
            return;
        }
        String mac_adress = advertisedDevice->getAddress().toString().c_str();
        mac_adress.toUpperCase();
        if (_opt->trace & INFO_BLE_ADS) {
            LOGD("BLE: {}", advertisedDevice->toString().c_str());
        }
        _config->ble_ads_accepted += 1;
        BLEdata["id"] = (char *)mac_adress.c_str();

        if (advertisedDevice->haveName())
            BLEdata["name"] = (char *)advertisedDevice->getName().c_str();

        if (advertisedDevice->haveManufacturerData()) {
            char *manufacturerdata = BLEUtils::buildHexData(
                NULL, (uint8_t *)advertisedDevice->getManufacturerData().data(),
                advertisedDevice->getManufacturerData().length());
            BLEdata["manufacturerdata"] = manufacturerdata;
            free(manufacturerdata);
        }

        if (advertisedDevice->haveRSSI())
            BLEdata["rssi"] = (int)advertisedDevice->getRSSI();

        if (advertisedDevice->haveTXPower())
            BLEdata["txpower"] = (int8_t)advertisedDevice->getTXPower();

        if (advertisedDevice->haveServiceData()) {
            int serviceDataCount = advertisedDevice->getServiceDataCount();
            for (int j = 0; j < serviceDataCount; j++) {
                std::string service_data =
                    convertServiceData(advertisedDevice->getServiceData(j));
                BLEdata["servicedata"] = (char *)service_data.c_str();
                std::string serviceDatauuid =
                    advertisedDevice->getServiceDataUUID(j).toString();
                BLEdata["servicedatauuid"] = (char *)serviceDatauuid.c_str();
            }
        }
        if (decoder.decodeBLEJson(BLEdata)) {
            // BLEdata.remove("manufacturerdata");
            BLEdata.remove("servicedata");
            BLEdata.remove("servicedatauuid");
            BLEdata.remove("type");
            BLEdata.remove("cidc");
            BLEdata.remove("acts");
            BLEdata.remove("cont");
            BLEdata.remove("track");
            if (_opt->trace & INFO_BLE_SENSORS) {
                Serial.printf("sensor %s: ", bp->name);
                serializeJson(BLEdata, Serial);
                Serial.println("");
            }
            // test for important sensors which should go into the global or custom state
            switch (bp->usage) {
                case BT_ENVELOPE:
                    if (bp->type == SENSOR_RUUVI) {
                        custom.env_temp_C  = BLEdata["tempc"];
                        custom.env_hum_pct = BLEdata["hum"];
                        custom.env_millis  = millis();
                    }
                    break;
                case BT_OAT:
                    if (bp->type == SENSOR_RUUVI) {
                        custom.oat_temp_C  = BLEdata["tempc"];
                        custom.oat_hum_pct = BLEdata["hum"];
                        custom.oat_millis  = millis();
                    }
                    break;
                case BT_FLOW1:
                    break;
                default:
                    break;
            }
        }
#ifdef TRACE2_PIN
        digitalWrite(TRACE2_PIN, LOW);
#endif
    }
    options_t *_opt;
    config_t *_config;
};

void setupBLE(options_t &opt, config_t &config) {
    // NimBLEDevice::setScanFilterMode(CONFIG_BTDM_SCAN_DUPL_TYPE_DATA);
    // NimBLEDevice::setScanDuplicateCacheSize(200);
    NimBLEDevice::init("");
    // NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    for (auto i = 0; i < NUM_BLESENSORS; i++) {
        BLEsensor_t *bp = &opt.blesensors[i];
        std::string mac;
        switch (bp->type) {
            case SENSOR_NONE:
                continue;
            default:
                config.nimble_adr[i] = NimBLEAddress(std::string(bp->blemac));
                break;
        }
    }
    // create new scan
    pBLEScan = NimBLEDevice::getScan();

    // no whitelist filtering  - done in onResult()
    pBLEScan->setFilterPolicy(BLE_HCI_SCAN_FILT_NO_WL);

    // Set the callback for when devices are discovered, no duplicates.
    pBLEScan->setScanCallbacks(new ScanCallbacks(&opt, &config), false);
    pBLEScan->setActiveScan(
        false);  // save power - we do not need the display name

    pBLEScan->setInterval(
        97);  // How often the scan occurs / switches channels; in milliseconds,
    pBLEScan->setWindow(
        37);  // How long to scan during the interval; in milliseconds.
    pBLEScan->setMaxResults(
        0);  // do not store the scan results, use callback only.
}

void BLEscanOnce(options_t &options, config_t &config) {
    // If an error occurs that stops the scan, it will be restarted here.
    if (pBLEScan->isScanning() == false) {
#ifdef TRACE1_PIN
        digitalWrite(TRACE1_PIN, HIGH);
#endif
        // Start scan with: duration = 1 seconds(forever), no scan end callback,
        // not a continuation of a previous scan.

        pBLEScan->start(0, false);
#ifdef TRACE1_PIN
        digitalWrite(TRACE1_PIN, LOW);
#endif
    }
}

#endif
