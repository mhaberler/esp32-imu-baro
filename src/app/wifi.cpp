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
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>

#include "Ticker.h"
#include "time.h"
#include <CmdBuffer.hpp>
#include <CmdCallback.hpp>
#include <CmdParser.hpp>

#include <NeoTeeStream.h>
#include <StreamUtils.h>
#include <WebSerial.h>
#include <WiFiMulti.h>
#include <WiFiUdp.h>

Ticker flushTicker;

AsyncWebServer server(80);

WiFiUDP udp;
WiFiMulti wifiMulti;

IPAddress teleplotSrv((uint32_t)0);

extern CmdCallback<NUM_COMMANDS> cmdCallback;
extern CmdBuffer<CMD_BUFSIZE> buffer;
extern CmdParser shell;

#define WS_BUF 1400
#define SERIAL_BUF 512

WriteBufferingStream bufferedSerialOut(Serial, SERIAL_BUF);
WriteBufferingStream bufferedWebSerialOut(WebSerial, WS_BUF);
Stream *streams[2] = {&bufferedSerialOut, &bufferedWebSerialOut};
NeoTeeStream tee(streams, sizeof(streams) / sizeof(streams[0]));

Fmt Console(&tee);

Teleplot teleplot;

const char *PARAM_MESSAGE = "message";

const char *ntp1 = "91.206.8.70";
const char *ntp2 = "78.41.116.149";
const char *ntp3 = "217.196.145.42";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 0;

void webserialCmdComplete(CmdParser &cmdParser, bool found) {
  if (!found) {
    const char *cmd = cmdParser.getCommand();
    if ((*cmd == '#') || (*cmd == ';')) {
      // a comment. ignore.
      return;
    }
    Console.fmt("command not found: '{}' \n", cmd);
    Console.fmt("type 'help' for a list of commands\n");
  }
}
void writeFunc(const uint8_t writeChar) { Console.write(writeChar); }

void recvMsg(uint8_t *data, size_t len) {
  for (int i = 0; i < len; i++) {
    cmdCallback.updateCmdProcessing(&shell, &buffer, data[i], writeFunc,
                                    webserialCmdComplete);
  }
  cmdCallback.updateCmdProcessing(&shell, &buffer, '\n', writeFunc,
                                  webserialCmdComplete);
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

int WifiSetup(options_t &opt) {
  int64_t millis_offset;
  time_t tt;

  flushTicker.attach_ms(500, []() {
    Console.flush();
    bufferedWebSerialOut.flush();
    WebSerial.flush();
    bufferedSerialOut.flush();
  });

  if (opt.num_ssid < 1) {
    Console.fmt("no WiFi access points configured, not starting WiFi\n");
    Console.fmt("using Serial for reporting\n");
    teleplot.begin(&Serial);
    return -1;
  }
  Console.fmt("starting wifi setup...\n");

  WiFi.persistent(false);
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_STA);

  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addServiceTxt("http", "tcp", "path", "/webserial");
  }
  for (auto i = 0; i < opt.num_ssid; i++) {
    Console.fmt("\tadding SSID {} pass {}\n", opt.ssids[i], opt.passwords[i]);
    wifiMulti.addAP(opt.ssids[i], opt.passwords[i]);
  }
  wl_status_t status = (wl_status_t)wifiMulti.run();
  switch (status) {
  case WL_DISCONNECTED: // no AP, use Serial
    Console.fmt(
        "no matching WiFi access points found - using Serial for reporting\n");
    teleplot.begin(&Serial);
    return -1;
    break;
  case WL_CONNECTED: // all good
    Console.fmt("IP {} GW {} netmask {} rssi {}\n",
                WiFi.localIP().toString().c_str(),
                WiFi.gatewayIP().toString().c_str(),
                WiFi.subnetMask().toString().c_str(), WiFi.RSSI());

    WiFi.printDiag(Console);

    delay(500); // NTP sometimes fails at startup, so give it some time
    configTime(gmtOffset_sec, daylightOffset_sec, ntp1, ntp2, ntp3);

    if (!getLocalTime(&config.timeinfo)) {
      Console.fmt("Failed to obtain time via NTP\n");
      tt = 0;
    } else {
      tt = mktime(&config.timeinfo);
    }
    config.timeinfo_millis = millis();
    config.timeSinceEpoch = ((int64_t)tt) * 1000;

    if (*opt.tpHost != '\0') {
      if (!WiFi.hostByName(opt.tpHost, teleplotSrv)) {
        Console.fmt("DNS resolve {} failed\n", opt.tpHost);
        break;
      }
      millis_offset = config.timeSinceEpoch - config.timeinfo_millis;
      tt = millis_offset / 1000;

      Console.fmt("Teleplot reftime={}", ctime(&tt));
      teleplot.begin(teleplotSrv, opt.tpPort, millis_offset);
    } else {
      // tpHost/tpPort unset - just use first result of mDNS scan
      int n = browseService("teleplot", "udp");
      if (n > -1) {
        config.tpHost = MDNS.IP(n - 1);
        config.tpPort = MDNS.port(n - 1);

        Console.fmt("using teleplot destination to {}:{}\n",
                    config.tpHost.toString().c_str(), config.tpPort);
        millis_offset = config.timeSinceEpoch - config.timeinfo_millis;
        time_t tmp = millis_offset / 1000;
        Console.fmt("NTP reftime={}", ctime(&tmp));

        teleplot.begin(config.tpHost, config.tpPort, millis_offset);
      } else {
        Console.fmt("tpdest not set and no result from mDNS discovery for "
                    "teleplot/udp\n");
        Console.fmt("using Serial port\n");
        millis_offset = config.timeSinceEpoch - config.timeinfo_millis;
        teleplot.begin(&Serial);
      }
    }
    // rest web setup
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "Hello, world");
    });

    // Send a GET request to <IP>/get?message=<message>
    server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
      String message;
      if (request->hasParam(PARAM_MESSAGE)) {
        message = request->getParam(PARAM_MESSAGE)->value();
      } else {
        message = "No message sent";
      }
      request->send(200, "text/plain", "Hello, GET: " + message);
    });

    // Send a POST request to <IP>/post with a form field message set to
    // <message>
    server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request) {
      String message;
      if (request->hasParam(PARAM_MESSAGE, true)) {
        message = request->getParam(PARAM_MESSAGE, true)->value();
      } else {
        message = "No message sent";
      }
      request->send(200, "text/plain", "Hello, POST: " + message);
    });

    server.onNotFound(notFound);

    // WebSerial is accessible at "<IP Address>/webserial" in browser
    WebSerial.begin(&server);
    WebSerial.msgCallback(recvMsg);

    server.begin();
    return 0;
    break;
  default:
    break;
  }
  return -1;
}

int browseService(const char *service, const char *proto) {
  Console.fmt("Browsing for service _{}._{}.local. ...\n", service, proto);
  int n = MDNS.queryService(service, proto);
  if (n == 0) {
    Console.fmt("no services found\n");
    return -1;
  } else {

    Console.fmt("{} service(s) found\n", n);
    for (int i = 0; i < n; ++i) {
      Console.fmt("{}: {} ({}:{})\n", i, MDNS.hostname(i).c_str(),
                  MDNS.IP(i).toString().c_str(), MDNS.port(i));
    }
    return n;
  }
}

#endif