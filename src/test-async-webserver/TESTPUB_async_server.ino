/******************************************************************************************

TEST_async_server

    Testing downloads of big files >100k

*******************************************************************************************/


//###################################################################################################
//set this to your wifi settings
    char                SSID[32]                = "mySSID";                 // 32   to be constrained
    char                password[63]            = "myPassword";             // 63   to be constrained
//###################################################################################################


#include <FS.h>                             // File System
#include <ESPAsyncWebServer.h>              // includes <WiFi.h>
#include <WiFi.h>                           // using WiFi
#include "SafeString.h"                     // avoiding the String class


//###################################################################################################
// FileSystem - activate one of FFat, LittleFS, or SPIFFS
// SPIFFS is useless for large files; at ~60% used space on 4MB Flash Watchdog triggers permanently
// make sure PARTITIONS settings match FS selection!

    //~#define        myFSCode     2    // use SPIFFS
    //~#define        myFSCode     1    // use FFat
    #define        myFSCode     3    // use LittleFS

#if   myFSCode == 1                             // using the FFat file system
    #include "FFat.h"
    fs::F_Fat       * myFS       = &FFat;
    #define           myFSName     "FFat"

#elif myFSCode == 2                             // using the SPIFFS file system
    #include "SPIFFS.h"
    fs::SPIFFSFS   *  myFS       = &SPIFFS;
    #define           myFSName     "SPIFFS"

#elif myFSCode == 3                             // using the LittleFS file system
    #include "LITTLEFS.h"
    fs::LITTLEFSFS  * myFS       = &LITTLEFS;
    #define           myFSName     "LittleFS"

#endif

//###################################################################################################

// names
#define             DATACAM                 "/data.cam"          // filename to store data in binary form

// useful constants
const uint32_t      mio                     = 1000000;           // = 1 million; used in micros() checks
const float         mibi                    = 1024.0f * 1024.0f; // = 1 048 576; to calculate MB from B
const float         kibi                    = 1024.0f;           // = 1024; to calculate kB from B

float               dataFilekB              = 0.0f;              // size of file data.cam in 1kB (1024B) (fractions allowed)
float               defaultdataFilekB       = 90.0f;             // filesize DATACAM = 90kB
bool                makingDataFile          = false;             // true during the making of a data file

// WiFi
int8_t              WiFi_STA_status         = -1;                // the connection status of WiFi for Station mode
bool                LostWifi                = false;

// FS
bool                bFSIsMounted            = false;             // true when the File System is mounted
uint8_t             FSCorruptState          = 0;                 // Corruption state of FS: 0:Ok, >0:corruption
const char *        FSCorruptType[]         = {  "OK"                                   // 0
                                               , "ERR:Found corrupt file"               // 1
                                               , "ERR:Failed to open directory '/'"     // 2
                                               , "ERR:'/' is not a directory"           // 3
                                               , "ERR:file is NULL"                     // 4
                                              };

// suggestion by BlueAndi
// https://github.com/me-no-dev/ESPAsyncWebServer/issues/984#issuecomment-846548444
static              AsyncWebServerRequest* deferredRequest = nullptr; 


//###################################################################################################
// Setup
//###################################################################################################

void setup() {

    Serial.begin(115200);
    Serial.setDebugOutput(true);         // allow Debug output on Serial
    SafeString::setOutput(Serial);       // activate SafeString output on Serial

    Serial.println("\nsetup begin ----------------------------------------------------------");

    // set extra debug output from ESP Core
    esp_log_level_set("*", ESP_LOG_VERBOSE); // set for everything & every level; but only I(info) messages seen

    // init file system
    initFileSystem();

    // initWifi
    initWIFI();

    // Init http service
    initWebServer();

    // Init Data File      
    initDataFile();

    Serial.println("setup end -----------------------------------------------------\n");
}


//###################################################################################################
// Main Loop
//###################################################################################################

void loop() {
    // check connection(s); no reconnection implemented
    if (!isWiFiconnected()){
        if (!LostWifi) {
            Serial.println("Lost WiFi");
            LostWifi = true;
        }
        else{
            Serial.print(".");
        }
        delay(300);
    }

    // make data file with new size if requested
    else if (dataFilekB > 0) {
        makeCAMFile((uint32_t)(dataFilekB * kibi));
        dataFilekB = 0;
    }

    else if (nullptr != deferredRequest){
        Serial.println("3 Loop: deferredRequest handling");                
        // deferredRequest->send(*myFS, "/data.cam", String(), true); // offer for saving
        deferredRequest->send(*myFS, "/data.cam", String(), false);   // display as webpage
        deferredRequest = nullptr;
    }

    else {
        // check for any commands from Serial Terminal
        handleTerminalCommands();
    }

    yield(); // needed?
}
