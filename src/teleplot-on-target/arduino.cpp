

// server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
//   AsyncWebServerResponse *response = request->beginResponse(SPIFFS,
//   "/index.html.gz", "text/html"); response->addHeader("Content-Encoding",
//   "gzip"); request->send(response);
// });

#include <Fmt.h>
#ifdef M5UNIFIED
#include <M5Unified.h>
#endif
#include "Teleplot.h"
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

WiFiUDP udp;
int browseService(const char *service, const char *proto);

Fmt Console(&Serial);
Teleplot teleplot;

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("- failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");

      Serial.print(file.name());
      time_t t = file.getLastWrite();
      struct tm *tmstruct = localtime(&t);
      Serial.printf("  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n",
                    (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1,
                    tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min,
                    tmstruct->tm_sec);

      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");

      Serial.print(file.size());
      time_t t = file.getLastWrite();
      struct tm *tmstruct = localtime(&t);
      Serial.printf("  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n",
                    (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1,
                    tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min,
                    tmstruct->tm_sec);
    }
    file = root.openNextFile();
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    yield();
  }
  if (LittleFS.begin()) {
    Console.fmt("LittleFS  total {}, used {} ({:.1f}%)\n",
                LittleFS.totalBytes(), LittleFS.usedBytes(),
                100.0 * LittleFS.usedBytes() / LittleFS.totalBytes());
    listDir(LittleFS, "/www", 2);

  } else {
    Console.fmt("LittleFS init failed\n");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Console.fmt("WiFi connection failed - using Serial\n");
    teleplot.begin(&Serial);
  } else {
    WiFi.printDiag(Serial);
    MDNS.begin("esp32");
    int n = browseService("teleplot", "udp");
    if (n > -1) {
      Console.fmt("using teleplot destination to {}:{}\n",
                  MDNS.IP(n - 1).toString().c_str(), MDNS.port(n - 1));
      teleplot.begin(MDNS.IP(n - 1), MDNS.port(n - 1));
    } else {
      Console.fmt("mDNS: no _teleplot._udp service found - using Serial\n");
      teleplot.begin(&Serial);
    }
  }

  float i = 0;
  int state_arr_length = 3;
  std::string state_arr[state_arr_length] = {"standing", "sitting", "walking"};

  int heights_arr_length = 6;
  double heights_arr[heights_arr_length] = {20, 5, 8, 4, 1, 2};

  for (;;) {
    // Use instanciated object
    teleplot.update("sin", sin(i), "km²");

    teleplot.update("cos", cos(i), "");
    teleplot.update("state", state_arr[rand() % state_arr_length], "", "t");

    teleplot.update3D(
        ShapeTeleplot("mysquare", "cube")
            .setCubeProperties(heights_arr[rand() % heights_arr_length])
            .setPos(sin(i) * 10, cos(i) * 10));

    delay(50);

    i += 0.1;
  }
}

void loop() { yield(); }

int browseService(const char *service, const char *proto) {
  Console.fmt("Browsing for service _{}._{}.local. ...\n", service, proto);
  int n = MDNS.queryService(service, proto);
  if (n == 0) {
    Console.fmt("no services found\n");
    return -1;
  } else {

    Console.fmt("{} service(s) found\n", n);
    for (int i = 0; i < n; ++i) {
      // Print details for each service found
      Console.fmt("{}: {} ({}:{})\n", i, MDNS.hostname(i).c_str(),
                  MDNS.IP(i).toString().c_str(), (int)MDNS.port(i));
    }
    return n;
  }
}
