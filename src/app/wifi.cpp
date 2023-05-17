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

#include <NTPClient.h>
#include <WiFiMulti.h>
#include <WiFiUdp.h>

WiFiUDP udp;
WiFiMulti wifiMulti;

NTPClient timeClient(udp, NTP_POOL);
IPAddress teleplotSrv((uint32_t)0);

Teleplot teleplot;

int WifiSetup(const options_t &opt) {
  int64_t millis_offset;
  time_t tt;

  if (opt.num_ssid < 1) {
    LOGD("no WiFi access points configured, not starting WiFi");
    LOGD("using Serial for reporting");
    teleplot.begin(&Serial);
    return -1;
  }
  LOGD("starting wifi setup...");

  WiFi.persistent(false);
  WiFi.setHostname(opt.hostname);
  WiFi.mode(WIFI_STA);

  // FIXME LOGD("---------WiFi.setSleep(false);");
  // WiFi.setSleep(false);

  if (MDNS.begin(opt.hostname)) {
    MDNS.addService("http", "tcp", 80);
    if (opt.run_webserial) {
      MDNS.addServiceTxt("http", "tcp", "path", opt.web_default_path);
    }
  }
  for (auto i = 0; i < opt.num_ssid; i++) {
    LOGD("\tadding SSID {} pass {}", opt.ssids[i], opt.passwords[i]);
    wifiMulti.addAP(opt.ssids[i], opt.passwords[i]);
  }
  wl_status_t status = (wl_status_t)wifiMulti.run();
  switch (status) {
  case WL_DISCONNECTED: // no AP, use Serial
    LOGD("no matching WiFi access points found - using Serial for reporting");
    teleplot.begin(&Serial);
    return -1;
    break;
  case WL_CONNECTED: // all good
    LOGD("IP {} GW {} netmask {} rssi {}", WiFi.localIP().toString().c_str(),
         WiFi.gatewayIP().toString().c_str(),
         WiFi.subnetMask().toString().c_str(), WiFi.RSSI());

    WiFi.printDiag(Console);
    LOGD("timeClient.forceUpdate: {}",
                    T2OK(timeClient.forceUpdate()));
    LOGD("timeClient says; {}", timeClient.getFormattedDate().c_str());

    config.timeSinceEpoch = ((int64_t)timeClient.getEpochTime()) * 1000;
    config.timeinfo_millis = millis();
    timeClient.end();

    if (*opt.tpHost != '\0') {
      if (!WiFi.hostByName(opt.tpHost, teleplotSrv)) {
        LOGD("DNS resolve {} failed", opt.tpHost);
        break;
      }
      millis_offset = config.timeSinceEpoch - config.timeinfo_millis;
      tt = millis_offset / 1000;

      LOGD("Teleplot reftime={}", ctime(&tt));
      teleplot.begin(teleplotSrv, opt.tpPort, millis_offset);
    } else {
      // tpHost/tpPort unset - just use first result of mDNS scan
      int n = browseService("teleplot", "udp");
      if (n > -1) {
        config.tpHost = MDNS.IP(n - 1);
        config.tpPort = MDNS.port(n - 1);

        LOGD("using teleplot destination to {}:{}",
             config.tpHost.toString().c_str(), config.tpPort);
        millis_offset = config.timeSinceEpoch - config.timeinfo_millis;
        time_t tmp = millis_offset / 1000;
        LOGD("NTP reftime={}", ctime(&tmp));

        teleplot.begin(config.tpHost, config.tpPort, millis_offset);
      } else {
        LOGD("tpdest not set and no result from mDNS discovery for "
             "teleplot/udp");
        LOGD("using Serial port");
        millis_offset = config.timeSinceEpoch - config.timeinfo_millis;
        teleplot.begin(&Serial);
      }
    }
    configureWebServer(opt);

    return 0;
    break;
  default:
    break;
  }
  return -1;
}

int browseService(const char *service, const char *proto) {
  LOGD("Browsing for service _{}._{}.local. ...", service, proto);
  int n = MDNS.queryService(service, proto);
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