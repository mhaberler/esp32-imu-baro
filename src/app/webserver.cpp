#include "defs.hpp"

#ifdef WEBSERVER
#ifdef M5UNIFIED
#include "M5Unified.h"
#include "SD.h"
#endif
#include <Arduino.h>
#ifdef ESP32
#include <AsyncTCP.h>
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include "Cache.hpp"
#include <CmdBuffer.hpp>
#include <CmdCallback.hpp>
#include <CmdParser.hpp>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <LittleFS.h>

#ifdef WEBSERIAL
#include <WebSerial.h>
#include <stdint.h>
#endif
#include "web.hpp"

const char *PARAM_MESSAGE = "message";

AsyncWebServer *server;
AsyncWebSocket *webSocket, *consoleSocket;

extern CmdCallback<NUM_COMMANDS> cmdCallback;
extern CmdBuffer<CMD_BUFSIZE> buffer;
extern CmdParser shell;

extern SdFat sdfat;

void webserialCmdComplete(CmdParser &cmdParser, bool found) {
  if (!found) {
    const char *cmd = cmdParser.getCommand();
    if ((*cmd == '#') || (*cmd == ';')) {
      // a comment. ignore.
      return;
    }
    LOGD("command not found: '{}' ", cmd);
    LOGD("type 'help' for a list of commands");
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

void wsRecvMsg(const void *data, size_t len) {
  serialConsole.fmt("websocket got: '{:.*s}'", len, data);
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info;
  client_t *ci;
  DeserializationError error;
  int n;

  switch (type) {
  case WS_EVT_DISCONNECT:
    LOGD("ws: disconnect", client->id());
    ci = (client_t *)client->userData();
    if (ci->clientMessage != NULL)
      delete ci->clientMessage;
    free(ci);
    break;

  case WS_EVT_CONNECT:
    LOGD("ws {}: connect from {}:{}", client->id(),
         client->remoteIP().toString().c_str(), client->remotePort());

    ci = (client_t *)heap_caps_malloc(sizeof(client_t), MALLOC_CAP_SPIRAM);
    if (ci == NULL) {
      LOGD("cant allocate client_t for client {}", client->id());
      client->close();
      break;
    }
    ci->clientMessage = new SpiRamJsonDocument(SPIRAM_JSON_DOCUMENT_SIZE);
    ci->clientMessage->clear();
    ci->subscription = DEFAULT_SUBSCRIPTION;
    ci->command = 0;
    ci->last_update = 0;
    client->setUserData((void *)ci);
    break;

  case WS_EVT_DATA:
    info = (AwsFrameInfo *)arg;
    LOGD("from client {}: received {}: {}", client->id(),
         info->opcode == WS_TEXT ? "text" : "binary", (const char *)data);

    ci = (client_t *)client->userData();

    error = deserializeJson(*ci->clientMessage, (const char *)data);
    if (error) {
      LOGD("deserializeJson() failed: {}", error.c_str());

      ci->clientMessage->clear();
      (*ci->clientMessage)["error"] = error.c_str();
      size_t len =
          serializeJson((*ci->clientMessage), config.txbuf, config.txbuf_size);
      client->text(config.txbuf, len);
      break;
    }

#ifdef FIXEDIT
    n = update_config_values(&config, ci->clientMessage);
#endif
    client->text(fmt::format("{{ \"items_changed\": {}}}", n).c_str());

    if ((*ci->clientMessage).containsKey("subscribe")) {
      ci->subscription =
          (*ci->clientMessage)["subscribe"].as<decltype(ci->subscription)>();

      if ((*ci->clientMessage).containsKey("interval")) {
        ci->interval =
            (*ci->clientMessage)["interval"].as<decltype(ci->interval)>();
      } else {
        if (ci->interval == 0)
          ci->interval = DEFAULT_CLIENT_INTERVAL;
      }
    }
    if ((*ci->clientMessage).containsKey("command")) {
      ci->command = (*ci->clientMessage)["command"].as<decltype(ci->command)>();
    }
    LOGD("client {}:  command={} subscribe={} interval={}", client->id(),
         ci->command, ci->subscription, ci->interval);
    break;

  default:
    break;
  }

  config.num_websocket_clients = webSocket->count();
}

void onConsoleSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                          AwsEventType type, void *arg, uint8_t *data,
                          size_t len) {
  AwsFrameInfo *info;

  switch (type) {
  case WS_EVT_DISCONNECT:
    LOGD("wsconsole disconnect client={}", client->id());
    break;
  case WS_EVT_CONNECT:
    LOGD("wsconsole client {} connect from {}:{}", client->id(),
         client->remoteIP().toString().c_str(), client->remotePort());
    // handleClientEvent(client_num, type);
    break;
  case WS_EVT_DATA:
    info = (AwsFrameInfo *)arg;
    LOGD("from client {}: received {}: {}", client->id(),
         info->opcode == WS_TEXT ? "text" : "binary", (const char *)data);
    // handleClientMessage(client->id(), type, data, len);
    break;
  default:
    break;
  }
}

class CaptiveRequestHandler : public AsyncWebHandler {
public:
  CaptiveRequestHandler() {}
  virtual ~CaptiveRequestHandler() {}

  bool canHandle(AsyncWebServerRequest *request) { return true; }

  void handleRequest(AsyncWebServerRequest *request) {
    request->redirect(INDEX_HTML);
  }
};

// Queue deferred(20, sizeof(void *));

void handle_deferred(void) {
  // AsyncWebServerRequest *request;
  // if (deferred.Dequeue(&request, 0)) {
  //   LOGD("deferred {}", request->pathArg(0).c_str());
  // }
}

bool configureWebServer(const options_t &opt, config_t &config) {
  if (true) {
    // if (opt.captive_handler) {
    server = new AsyncWebServer(opt.webserver_port);
    server->reset();
    webSocket = new AsyncWebSocket(String(opt.websocket_path));
    consoleSocket = new AsyncWebSocket(String(opt.consolesocket_path));

    webSocket->onEvent(onWebSocketEvent);
    consoleSocket->onEvent(onConsoleSocketEvent);

    server->addHandler(webSocket);
    server->addHandler(consoleSocket);

    // server->on("^(\\/maps\\/.+)$", HTTP_GET,
    //            [](AsyncWebServerRequest *request) {
    //              LOGD("maps {}", request->pathArg(0).c_str());
    //              deferred.Enqueue(request);
    //            });
#ifdef SDFAT
    if (config.sdfat_healthy) {
      server->serveStatic("/", &sdfat, "/", "");
    }
#endif
#ifdef LITTLEFS
    if (config.lfs_healthy) {
      server->serveStatic(opt.littlefs_static_path, LittleFS, "/");
    }
#endif
    //    .setDefaultFile(opt.web_default_path);

#if 0
  // server->on("^\\/sdfile(\\/.+)$", HTTP_GET,
  //            [](AsyncWebServerRequest *request) {
  //              LOGD("SD {}", request->pathArg(0).c_str());
  //              request->send(SD, request->pathArg(0), "", false, NULL);
  //            });
  // server->on("^\\/sddownload(\\/.+)$", HTTP_GET,
  //            [](AsyncWebServerRequest *request) {
  //              LOGD("SD {}", request->pathArg(0).c_str());
  //              request->send(SD, request->pathArg(0), "", true, NULL);
  //            });

  // server->serveStatic(SD_MOUNTPOINT, SD, SD_MOUNTPOINT);

  server->on("^\\/file(\\/.+)$", HTTP_GET, [](AsyncWebServerRequest *request) {
    LOGD("LittleFS file {}", request->pathArg(0).c_str());
    request->send(LittleFS, request->pathArg(0), "", false, NULL);
  });
  server->on("^\\/download(\\/.+)$", HTTP_GET,
             [](AsyncWebServerRequest *request) {
               LOGD("LittleFS download {}", request->pathArg(0).c_str());
               request->send(LittleFS, request->pathArg(0), "", true, NULL);
             });


  // Send a GET request to <IP>/get?message=<message>
  server->on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
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
  server->on("/post", HTTP_POST, [](AsyncWebServerRequest *request) {
    String message;
    if (request->hasParam(PARAM_MESSAGE, true)) {
      message = request->getParam(PARAM_MESSAGE, true)->value();
    } else {
      message = "No message sent";
    }
    request->send(200, "text/plain", "Hello, POST: " + message);
  });

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
#endif

    server->onNotFound(notFound);

    // #ifdef WEBSERIAL
    //   // WebSerial is accessible at "<IP Address>/webserial" in browser
    //   WebSerial.begin(server);
    //   WebSerial.msgCallback(recvMsg);
    // #endif

    server->begin();
  } else {
    server = new AsyncWebServer(opt.webserver_port);

    server->onNotFound(notFound);
    server->serveStatic("/", &sdfat, "/", "");
    server->addHandler(new CaptiveRequestHandler());
    server->begin();
  }

  return true;
}
#endif // WEBSERVER