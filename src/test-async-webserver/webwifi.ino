/*
 * WiFi
 */


void initWIFI() {

    auto t0 = micros();
    Serial.println("Initializing WIFI in Station Mode");

    Serial.println("set disconnect -> true");
    WiFi.disconnect(true);

    Serial.println("set mode       -> WIFI_OFF");
    WiFi.mode(WIFI_OFF);                            // set NULL Mode
    delay(10); // setting NULL needs some time? maybe, 10ms seems enough

    Serial.println("set mode       -> WIFI_STA");
    WiFi.mode(WIFI_STA);                            // set STA Mode
    delay(10); // setting STA needs some time? yes, 10ms seems enough

    // Do up to retries rounds of connection attempts
    // with ESP Core 1.0.4 Double-Hits were almost always required
    // with ESP Core 1.0.6 all Single-Hits worked
    uint8_t i       = 0;
    uint8_t retries = 2;
    const char *  dash20  = "-------------------- ";
    const char *  wstatus = "FAILURE";
    cSF(msg, 550);
    for (i = 0; i < retries; i++){
        msg.clear();
        msg.printf("%sWiFi Connection round #%i", dash20, i);
        Serial.println(msg);

        WiFi_STA_status = getWifiConnection(i);
        //~Serial.printf("WiFi_STA_status: %i \n", WiFi_STA_status);

        if      (WiFi_STA_status == 3) {i++;  wstatus = "SUCCESS";                  break;}    // success
        else if (WiFi_STA_status == 1) {i++;  wstatus = "FAILURE - SSID NOT AVAIL"; break;}    // no ssid of the given name
    }
    msg.clear();
    msg.printf("%sWiFi Initialization ended after %i connection round(s) with '%s'", dash20, i , wstatus);
    Serial.println(msg);

    if (WiFi_STA_status == 3){                       // SUCCESS, have connection
        msg.clear();
        msg.printf("connected: SSID:'%s', RSSI:%i, Hostname:'%s', IP:%s, BSSID:%s, dur:%0.0fms",
                        SSID,
                        WiFi.RSSI(),                 // the current RSSI (Received Signal Strength in dBm)
                        WiFi.getHostname(),
                        WiFi.localIP().toString().c_str(),
                        WiFi.BSSIDstr().c_str(),
                        (micros() - t0) / 1000.0
                   );
        Serial.println(msg);
    }
    else{
        // wifi disconnect and mode off
        WiFi.disconnect();                          // to avoid the system retrying
        WiFi.mode(WIFI_OFF);                        // set NULL Mode
        Serial.println("WiFi Station Mode set to OFF");
    }
}


int getWifiConnection(uint8_t round){

    uint8_t     counter         = 0;                    // counting number of connection checks
    int         checkinterval   = 100;                  // delay in ms between checks for Wifi connection
    auto        t0              = millis();             // to limit duration of status checking
    int         connstatus      = -1;                   // the connection status; -1 = not set

    cSF(msg, 2000);

    msg.printf("getWifiConnection: WiFi.begin(SSID: '%s', pw: '%s')", SSID, "***");
    Serial.println(msg);

    WiFi.begin(SSID, password);                 // WPA --> with password

    msg = "   Check[ms:Status]: ";
    while ((millis() - t0) < 5000 ){            // check no longer than 5 sec
        uint16_t wifistatus = WiFi.status();
        // Serial.printf("wifistatus: %i\n", wifistatus);
        msg.printf("%i:%i ", counter * checkinterval, wifistatus);
        if (wifistatus == WL_CONNECTED)                     {msg += "CONNECTION";       connstatus = 3; break;}
        if (wifistatus == WL_CONNECT_FAILED and round == 0) {msg += "FAILURE";          connstatus = 4; break;}
        if (wifistatus == WL_NO_SSID_AVAIL)                 {msg += "SSID NOT AVAIL";   connstatus = 1; break;}
        counter++;
        delay(checkinterval);
    }
    Serial.println(msg);

    return connstatus;
}


bool isWiFiconnected(){

    try{
        return (WiFi.status() == WL_CONNECTED);
    }
    catch (...) {
        return false;
    }
}
