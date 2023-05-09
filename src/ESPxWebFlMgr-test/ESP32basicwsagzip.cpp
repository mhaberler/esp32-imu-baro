#ifdef M5UNIFIED
#include <M5Unified.h>
#include <SD.h>
#endif

#include <FS.h>
#include <LittleFS.h>
#include <WebServer.h>
#include <WiFi.h>
// getting access to the nice mime-type-table and getContentType()
#include <detail/RequestHandlersImpl.h>

#include <ESPxWebFlMgr.h>
// #include <SD_MMC.h>

const word sd_port = 8080;
const word lfs_port = 8081;

const String hlssid = WIFI_SSID;
const String hlpwd = WIFI_PASSWORD;

void setupWebserver(void);
void loopWebServer(void);
void webserverServerNotFound(void);

ESPxWebFlMgr
    lfs_filemgr(lfs_port,
                LittleFS); // we want a different port than the webserver
#ifdef USE_SD
ESPxWebFlMgr sd_filemgr(sd_port, SD);
#endif


void setup(void) {
#ifdef M5UNIFIED
  auto cfg = M5.config();
  cfg.serial_baudrate =
      BAUD; // default=115200. if "Serial" is not needed, set it to 0.
  cfg.led_brightness = 128; // default= 0. system LED brightness (0=off /
                            // 255=max) (※ not NeoPixel)
  M5.begin(cfg);
#else
  Serial.begin(BAUD);
#endif

  /* Wait for the Serial Monitor */
  while (!Serial) {
    yield();
  }

  Serial.println(
      "\n\nESP32WebFlMgr Demo basicwsagzip"); // BASIC and WebServer And GZIPper

  if (!LittleFS.begin(true)) { // Format if failed.
    Serial.println("LittleFS Mount Failed");
    return;
  }
  Serial.println("LittleFS mounted.");


#ifdef USE_SD
#ifdef M5UNIFIED
  while (false == SD.begin(GPIO_NUM_4, SPI, 25000000, "/sdcard", 5, true)) {
    delay(500);
    Serial.println("SD waiting...");
  }
  Serial.println("SD mounted.");
#else
  if (!SD.begin("/sdcard", true)) {
    Serial.println("SD Mount Failed");
  }
#endif
#endif
  // login into WiFi
  WiFi.begin(hlssid.c_str(), hlpwd.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    delay(1);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Open SD card Filemanager with http://");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.print(sd_port);
    Serial.print("/");
    Serial.println();

    Serial.print("Open LittleFS card Filemanager with http://");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.print(lfs_port);
    Serial.print("/");

    Serial.println();
    Serial.print("Webserver is at http://");
    Serial.print(WiFi.localIP());
    Serial.print("/");
    Serial.println();
  }

  setupWebserver();
  // sd_filemgr.setSysFileStartPattern("/");
#ifdef USE_SD
  sd_filemgr.begin();
#endif
  lfs_filemgr.begin();
}

void loop() {
  sd_filemgr.handleClient();
  lfs_filemgr.handleClient();
  loopWebServer();
}
