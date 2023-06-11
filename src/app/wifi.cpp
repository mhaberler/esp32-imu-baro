#include "defs.hpp"

#ifdef WIFI
#include <Arduino.h>
#ifdef ESP32
#include <AsyncTCP.h>
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include <ESPmDNS.h>

// #include "time.h"
#include <CmdBuffer.hpp>
#include <CmdCallback.hpp>
#include <CmdParser.hpp>

#include <DNSServer.h>
#include <NTPClient.h>
#include <WiFiMulti.h>
#include <WiFiUdp.h>

WiFiUDP udp;
WiFiMulti wifiMulti;
DNSServer dnsServer;

NTPClient timeClient(udp, NTP_POOL);
IPAddress teleplotSrv((uint32_t)0);

Teleplot teleplot;
extern bool run_dns;

bool WifiSetup(const options_t &opt, config_t &config) {
    time_t tt, tmp;
    int n;

    if (opt.num_ssid < 1) {
        LOGD("no WiFi access points configured");
        if (strlen(opt.ap_ssid)) {
            LOGD("switching to AP mode using ssid {} password {}", opt.ap_ssid,
                 opt.ap_password);
            WiFi.mode(WIFI_AP);
            WiFi.softAP(opt.ap_ssid, opt.ap_password);
            WiFi.softAPsetHostname(opt.hostname);
            dnsServer.start(53, "*", WiFi.softAPIP());
            run_dns = true;
            LOGD("IP address: {}", WiFi.softAPIP().toString().c_str());
        }
    } else {
        LOGD("starting wifi client setup...");

        WiFi.persistent(false);
        WiFi.setHostname(opt.hostname);
        WiFi.mode(WIFI_STA);

        for (auto i = 0; i < opt.num_ssid; i++) {
            LOGD("\tadding SSID {} pass {}", opt.ssids[i], opt.passwords[i]);
            wifiMulti.addAP(opt.ssids[i], opt.passwords[i]);
        }
        wl_status_t status = (wl_status_t)wifiMulti.run();
        switch (status) {
            case WL_DISCONNECTED:  // no AP, use Serial
                LOGD(
                    "no matching WiFi access points found - using Serial for "
                    "reporting");
                teleplot.begin(&Serial);
                return -1;
                break;
            case WL_CONNECTED:  // all good
                config.wifi_avail = true;
                LOGD("IP {} GW {} netmask {} rssi {}",
                     WiFi.localIP().toString().c_str(),
                     WiFi.gatewayIP().toString().c_str(),
                     WiFi.subnetMask().toString().c_str(), WiFi.RSSI());
                WiFi.printDiag(Console);

                LOGD("timeClient.forceUpdate: {}",
                     T2OK(timeClient.forceUpdate()));
                config.timeSinceEpoch =
                    ((int64_t)timeClient.getEpochTime()) * 1000;
                config.timeinfo_millis = millis();
                timeClient.end();
                config.ntp_time_set = true;
                // todo: ping client - check internet connectivity
                config.millis_offset =
                    config.timeSinceEpoch - config.timeinfo_millis;
                tmp = config.millis_offset / 1000;
                LOGD("NTP reftime={}", ctime(&tmp));
                config.millis_offset =
                    config.timeSinceEpoch - config.timeinfo_millis;
                tt = config.millis_offset / 1000;
                LOGD("Teleplot reftime={}", ctime(&tt));

                if (*opt.tpHost != '\0') {
                    // tpdest was manually specified
                    if (!WiFi.hostByName(opt.tpHost, teleplotSrv)) {
                        LOGD("DNS resolve {} failed", opt.tpHost);
                    } else {
                        LOGD(
                            "using manually defined teleplot destination {}:{}",
                            teleplotSrv.toString().c_str(), opt.tpPort);
                        config.currentTpHost = teleplotSrv;
                        config.currentTpPort = opt.tpPort;
                        teleplot.begin(teleplotSrv, opt.tpPort,
                                       config.millis_offset);
                    }
                } else {
                    if (!MDNS.begin(opt.hostname)) {
                        LOGE("MDNS.begin({}) failed", opt.hostname);
                    }
                    // tpHost/tpPort unset - do an mDNS scan 
                    // and take IP address in "our" net
                    n = browseService("teleplot", "udp");
                    if (n > -1) {
                        LOGD("browseService n={}", n);
                        IPAddress tpHost;
                        for (auto i = 0; i < n; i++) {
                            IPAddress tpHost = MDNS.IP(i);
                            LOGD("checking {} vs {} mask {}",
                                 tpHost.toString().c_str(),
                                 WiFi.localIP().toString().c_str(),
                                 WiFi.subnetMask().toString().c_str());
                            if ((tpHost & WiFi.subnetMask()) ==
                                (WiFi.localIP() & WiFi.subnetMask())) {
                                // this is in "our" net
                                uint16_t tpPort = MDNS.port(n - 1);
                                LOGD("using mDNS teleplot destination {}:{}",
                                     tpHost.toString().c_str(), tpPort);
                                teleplot.begin(tpHost, tpPort,
                                               config.millis_offset);
                                config.currentTpHost = tpHost;
                                config.currentTpPort = tpPort;
                                break;
                            }
                        }
                    } else {
                        LOGD(
                            "tpdest not set and no result from mDNS discovery "
                            "for "
                            "teleplot/udp");
                        LOGD("using teleplot on Serial port");
                        teleplot.begin(&Serial);
                    }
                }
                break;
            default:
                break;
        }
    }
    if (MDNS.begin(opt.hostname)) {
        MDNS.addService("http", "tcp", 80);
        if (opt.run_webserial) {
            MDNS.addServiceTxt("http", "tcp", "path", opt.web_default_path);
        }
    }
    return true;
}

int browseService(const char *service, const char *proto) {
    LOGD("Browsing for service _{}._{}.local. ...", service, proto);
    int n = MDNS.queryService(service, proto);
    if (n == 0) {
        // retry - library has fixed 3sec timeout and sometimes this is too slow
        LOGD("retrying for service _{}._{}.local. ...", service, proto);
        n = MDNS.queryService(service, proto);
    }
    if (n == 0) {
        LOGD("no services found");
        return -1;
    } else {
        LOGD("{} service(s) found", n);
        for (int i = 0; i < n; ++i) {
            LOGD("{}: {} ({}:{})", i, MDNS.hostname(i).c_str(),
                 MDNS.IP(i).toString().c_str(), MDNS.port(i));
        }
        return n;
    }
}

#endif