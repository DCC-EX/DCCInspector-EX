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

#include "Stringbuilder.h"
#include "DccStatistics.h"

class HttpManagerClass {
  public:
    // Function to initialise the object, connect to WiFi and start HTTP server.
    bool begin(const char *ssid, const char *password, const char *dnsName);
    // Function to provide a buffer of dynamic data (decoded DCC packets).
    void setBuffer(char *buffer);
    // Function to write statistics, formatted as HTML, to a print stream.
    void writeHtmlStatistics(Statistics &stats, bool showCpuStats, bool showBitLengths);
    // Function to be called in loop() function to keep things going.
    void process();
    // Function to get null-terminated string containing html data.
    char *getHtmlString() { 
      return sbHtml.getString();
    }

  private:  
    // Flag whether connected or not.
    bool connected = false; 
    // Buffer for generating HTML
    char buffer[4000] = "";
    StringBuilder sbHtml = StringBuilder(buffer, sizeof(buffer));

} /* class HttpManagerClass */;

// Singleton class instance.
extern HttpManagerClass HttpManager;

#endif
