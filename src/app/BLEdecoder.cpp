#ifdef BLE_DECODER

#include "defs.hpp"

#include "NimBLEDevice.h"
#include "decoder.h"

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

    void onResult(BLEAdvertisedDevice *advertisedDevice) {
#ifdef TRACE2_PIN
        digitalWrite(TRACE2_PIN, HIGH);
#endif
        _config->ble_ads += 1;
        if (_opt->trace & INFO_BLE_ADS) {
            LOGD("BLE: {}", advertisedDevice->toString().c_str());
        }
        JsonObject BLEdata = doc.to<JsonObject>();
        String mac_adress  = advertisedDevice->getAddress().toString().c_str();
        mac_adress.toUpperCase();
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
                Serial.print("TheengsDecoder found device: ");
                serializeJson(BLEdata, Serial);
                Serial.println("");
            }

            // test for envelope mac address and record temp if so
            
        }
#ifdef TRACE2_PIN
        digitalWrite(TRACE2_PIN, LOW);
#endif
    }
    options_t *_opt;
    config_t *_config;
};

void setupBLE(options_t &opt, config_t &config) {
    NimBLEDevice::setScanFilterMode(CONFIG_BTDM_SCAN_DUPL_TYPE_DATA);
    NimBLEDevice::setScanDuplicateCacheSize(200);
    NimBLEDevice::init("");
    //   NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    pBLEScan = NimBLEDevice::getScan();  // create new scan
    // Set the callback for when devices are discovered, no duplicates.
    pBLEScan->setScanCallbacks(new ScanCallbacks(&opt, &config), false);
    pBLEScan->setActiveScan(true);  // Set active scanning, this will get more
                                    // data from the advertiser.
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
