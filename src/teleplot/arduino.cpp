
#include <Fmt.h>
#ifdef M5UNIFIED
#include <M5Unified.h>
#endif
#include "Teleplot.h"
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <stdlib.h>

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

WiFiUDP udp;
int browseService(const char *service, const char *proto);

Fmt Console(&Serial);
Teleplot teleplot;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    yield();
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    LOGD("WiFi connection failed - using Serial\n");
    teleplot.begin(&Serial);
  } else {
    WiFi.printDiag(Serial);
    MDNS.begin("esp32");
    int n = browseService("teleplot", "udp");
    if (n > -1) {
      LOGD("using teleplot destination to {}:{}",
                  MDNS.IP(n - 1).toString().c_str(), MDNS.port(n - 1));
      teleplot.begin(MDNS.IP(n - 1), MDNS.port(n - 1));
    } else {
      LOGD("mDNS: no _teleplot._udp service found - using Serial\n");
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
  LOGD("Browsing for service _{}._{}.local. ...", service, proto);
  int n = MDNS.queryService(service, proto);
  if (n == 0) {
    LOGD("no services found\n");
    return -1;
  } else {

    LOGD("{} service(s) found", n);
    for (int i = 0; i < n; ++i) {
      // Print details for each service found
      LOGD("{}: {} ({}:{})", i, MDNS.hostname(i).c_str(),
                  MDNS.IP(i).toString().c_str(), (int)MDNS.port(i));
    }
    return n;
  }
}
