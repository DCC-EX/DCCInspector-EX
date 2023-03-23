/* Copyright (c) 2021 Neil McKechnie
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/*
 * HttpManager class definition - include file.
 * 
 * Encapsulates the WiFI and Web Server functionality of the DCC inspector program.
 * 
 * HTTPMANAGER ISN'T SUPPORTED ON NANO, UNO or MEGA because of lack of memory.
 */

#ifndef httpmanager_h
#define httpmanager_h

#include "Config.h"

#if defined(ESP8266) 
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
  #define WebServer ESP8266WebServer
  #include <ESP8266mDNS.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <WebServer.h>
  #include <ESPmDNS.h>
  #include <esp_wps.h>
#endif

#include "StringBuilder.h"
#include "DCCStatistics.h"

class HttpManagerClass {
  public:
    // Function to initialise the object, connect to WiFi and start HTTP server.
    bool begin(const char *ssid, const char *password, const char *dnsName);
    // Function to provide a buffer of dynamic data (decoded DCC packets).
    void setBuffer(char *buffer);
    // Function to write statistics, formatted as HTML, to a print stream.
    void writeHtmlStatistics(Statistics &stats);
    // Function to be called in loop() function to keep things going.
    void process();
    // Function to get null-terminated string containing html data.
    void getIP();
    // Function to get IP address if connected
    char *getHtmlString() { 
      return sbHtml.getString();
    }

  private:  
    static void handleRoot();
    static void processArguments();
    static void handleNotFound();
    static void handleData();
    #if defined(ESP32)
    static void WiFiEvent (arduino_event_id_t event);
    #endif
    // Flag whether connected or not.
    bool connected = false; 
    // Buffer for generating HTML
    char buffer[4000] = "";
    StringBuilder sbHtml = StringBuilder(buffer, sizeof(buffer));
    static char *bufferPointer;
    static WebServer server;
} /* class HttpManagerClass */;

// Singleton class instance.
extern HttpManagerClass HttpManager;

#endif
