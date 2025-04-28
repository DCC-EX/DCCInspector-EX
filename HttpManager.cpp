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
 * HttpManager class definition - methods and static data.
 * 
 * Encapsulates the WiFI and Web Server functionality of the DCC inspector program.
 * 
 */

#include <Arduino.h>
extern byte strictMode;
extern bool filterInput;

#include "Config.h"

#ifdef USE_HTTPSERVER

#include "HttpManager.h"

#include "OledDisplay.h"
#include "DCCStatistics.h"

#if defined(ESP32) 
  #define PLATFORM "ESP32"
#elif defined(ESP8266)
  #define PLATFORM "ESP8266"
#endif

// Buffer pointer reference.  This is the buffer of dynamic text sent to the web server client.
char *HttpManagerClass::bufferPointer = 0;

// Web server object.
WebServer HttpManagerClass::server;

// Function to handle request for the root web page (http://server/).
//  It sends a static web page that includes an updating IFRAME where the dynamic data
//  will be displayed.
void HttpManagerClass::handleRoot() {
#if defined(LED_BUILTIN)
  digitalWrite(LED_BUILTIN, 1);
#endif
  processArguments();
  int refreshTime = DCCStatistics.getRefreshTime();
  String temp = 
    "<html>\r\n"
      "<script>\r\n"
        "window.setInterval('reloadIFrame();', 2000);\r\n"
        "function reloadIFrame() {\r\n"
          "document.getElementById('data').src='/data';\r\n"
        "}\r\n"
      "</script>\r\n"
      "<head>\r\n"
        "<title>DCC Diagnostics - Bit analysis</title>\r\n"
        "<style>\r\n"
          "body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\r\n"
        "</style>\r\n"
      "</head>\r\n"
      "<body>\r\n"
        "<h1>DCC Diagnostics</h1>\r\n"
        "<p>Diagnostic information from " PLATFORM " server:</p>\r\n"
        "<form action=\"/\" method=\"post\">\r\n"
          "<label for=\"nmraMode\">NMRA Validation level:</label>\r\n"
          "<select name=\"s\"> id=\"nmraMode\">\r\n"
            "<option value=0" + String(strictMode==0 ? " selected" : "") + ">No validation</option>\r\n"
            "<option value=1" + String(strictMode==1 ? " selected" : "") + ">Decoder (+/- 6us)</option>\r\n"
            "<option value=2" + String(strictMode==2 ? " selected" : "") + ">Strict (+/- 3us)</option>\r\n"
          "</select>\r\n"
          "<br>\r\n"
          "<label for=\"refreshTime\">Sample time</label>\r\n"
          "<select name=\"r\"> id=\"refreshTime\">\r\n"
            "<option value=1" + String(refreshTime==1 ? " selected" : "") + ">1 second</option>\r\n"
            "<option value=2" + String(refreshTime==2 ? " selected" : "") + ">2 seconds</option>\r\n"
            "<option value=4" + String(refreshTime==4 ? " selected" : "") + ">4 seconds</option>\r\n"
            "<option value=8" + String(refreshTime==8 ? " selected" : "") + ">8 seconds</option>\r\n"
            "<option value=16" + String(refreshTime==16 ? " selected" : "") + ">16 seconds</option\r\n>"
            "<option value=32" + String(refreshTime==32 ? " selected" : "") + ">32 seconds</option\r\n>"
            "<option value=64" + String(refreshTime==64 ? " selected" : "") + ">64 seconds</option\r\n>"
          "</select>\r\n"
          "<br>\r\n"
          "<label for=\"filterGlitches\">Filter Input Glitches</label>\r\n"
          "<select name=\"f\"> id=\"filterGlitches\">\r\n"
            "<option value=0" + String(filterInput==0 ? " selected" : "") + ">Off</option>\r\n"
            "<option value=1" + String(filterInput==1 ? " selected" : "") + ">On</option>\r\n"
          "</select>"
          "<br>\r\n"
          "<button>Apply Settings</button>\r\n"
        "</form>\r\n"
        "<p><iframe id=\"data\" src=\"/data\" width=500 "
        "   onload=\"this.style.height=(this.contentWindow.document.body.scrollHeight)+'px';\">\r\n"
        "</iframe></p>\r\n"
      "</body>\r\n"
    "</html>";
  server.send(200, "text/html", temp);
  temp = "";  // release space held

#if defined(LED_BUILTIN)
  digitalWrite(LED_BUILTIN, 0);
#endif
}

// Function to handle the request for dynamic data (http://server/data).
void HttpManagerClass::handleData() {
#if defined(LED_BUILTIN)
  digitalWrite(LED_BUILTIN, 1);
#endif
  server.send(200, "text/html", HttpManager.getHtmlString());
#if defined(LED_BUILTIN)
  digitalWrite(LED_BUILTIN, 0);
#endif
}

// Function to handle any other requests.  Returns "404 Not Found".
void HttpManagerClass::handleNotFound() {
#if defined(LED_BUILTIN)
  digitalWrite(LED_BUILTIN, 1);
#endif
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, "text/plain", message);
#if defined(LED_BUILTIN)
  digitalWrite(LED_BUILTIN, 0);
#endif
}

void HttpManagerClass::processArguments() {
  if (server.hasArg("r"))
    DCCStatistics.setRefreshTime(server.arg("r").toInt());
  if (server.hasArg("s")) 
    strictMode = server.arg("s").toInt();
  if (server.hasArg("f"))
    filterInput = server.arg("f").toInt();
}

#if defined(ESP32)
void HttpManagerClass::WiFiEvent (arduino_event_id_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
      Serial.print(F("[WPS Successful]"));
      esp_wifi_wps_disable ();
      WiFi.begin ();
      break;
    // case SYSTEM_EVENT_STA_DISCONNECTED:
    //   Serial.print(F("[WiFi Disconnected, reconnecting]"));
    //   WiFi.reconnect();
    //   break;
    // case SYSTEM_EVENT_STA_WPS_ER_FAILED:
    //   Serial.println("WPS Failed, retrying");
    //   // esp_wifi_wps_disable();
    //   esp_wifi_wps_start(0);
    //   break;
    // case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
    //   Serial.println("WPS Timed out, retrying");
    //   // esp_wifi_wps_disable();
    //   // esp_wifi_wps_enable(&config);
    //   esp_wifi_wps_start(0);
    //   break;
    default:
      break;
  }
}
#endif

// Function to initialise the object.  It connects to the WiFi access point, 
//  registers an mDNS name (e.g. ESP.local) and sets up the web server.
//  By preference, it will connect using the same SSID and password as last time.
//  If this fails, it tries WPS.  If that fails, it tries the configured SSID/Password.
//  If nothing works, the WiFi/HTTP capabilities are disabled.
bool HttpManagerClass::begin(const char *ssid, const char *password, const char *dnsName) {
  // Configuration for WiFi wps mode.
  #if defined(ESP32)
  esp_wps_config_t config;
  memset(&config, 0, sizeof(config));
  config.wps_type = WPS_TYPE_PBC;

  WiFi.onEvent (WiFiEvent);
  #endif

  int connectMode = 0;
  
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  int waitTime = 0;
  while (WiFi.status() != WL_CONNECTED) {
    switch (connectMode++) {
      case 0:  // Try persistent credentials
        WiFi.setAutoConnect(true);
        WiFi.begin();
        Serial.println();
        // Wait for connection
        Serial.print(F("Connecting to cached WiFi "));
        #if defined(USE_OLED)
        OledDisplay.append("Connecting to WiFi");
        #endif
        waitTime = 10;  // 10 second timeout.
        break;
      case 1:  // Try WPS
        Serial.println();
        WiFi.disconnect();
        Serial.print(F("Connecting via WPS "));
        #if defined(USE_OLED)
        OledDisplay.append("Trying WPS");
        #endif
        #if defined(ESP32)
        esp_wifi_wps_enable(&config);
        esp_wifi_wps_start(0);
        #elif defined(ESP8266)
        WiFi.beginWPSConfig();
        #endif
        
        waitTime = 20;  // 20 second timeout
        break;
      case 2: // Try nominated user/pass (if not blank)
        if (ssid != 0 && ssid[0] != 0) {
          WiFi.begin(ssid, password);
          Serial.println();
          Serial.print(F("Connecting to \""));
          Serial.print(ssid);
          Serial.print(F("\" "));
          waitTime = 10;  // 10 second timeout
        } else
          waitTime = 0;
        break;
      case 3: // Exhausted all possibilities, carry on.
        Serial.println();
        Serial.println(F("Can't connect."));
        #if defined(USE_OLED)
        OledDisplay.append("Can't connect");
        #endif
        return false;
    }
    // Wait for connection for 15 seconds.
    while (waitTime-- > 0) {
      if (WiFi.status() == WL_CONNECTED)
        break;
      
      delay(1000);
      Serial.print('.');
    }
    #if defined(ESP32)
    esp_wifi_wps_disable();
    #endif
  }

  connected = true;
  
  Serial.println();
  Serial.print(F("Connected to \""));
  Serial.print(WiFi.SSID());
  Serial.print(F("\", IP address: "));
  Serial.println(WiFi.localIP());
  #if defined(USE_OLED)
  OledDisplay.append("Connected!");
  #endif

  if (MDNS.begin(dnsName)) {
    Serial.println(F("MDNS responder started"));
  }
  
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println(F("HTTP server started"));

  return true;
}

// Function to set the pointer to dynamic data to be sent to the
//  web client on request.
void HttpManagerClass::setBuffer(char *bp) {
  bufferPointer = bp;
}

// Function to be called from the loop() function to do all the
//  regular processing related to the class.
void HttpManagerClass::process() {
  if (connected)
    server.handleClient();
}

// Function to display the IP Address of the server if connected
void HttpManagerClass::getIP() {
  if (connected){
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
  }
  else {
    Serial.println("Not Connected!"); 
  }
}

//=======================================================================
// WriteHtmlStatistics writes the last set of statistics to a print stream.
void HttpManagerClass::writeHtmlStatistics(Statistics &lastStats) 
{
  const char *rowStart = "<tr><td>";
  const char *cellDiv = "</td><td>";
  const char *rowEnd = "</td></tr>";
  sbHtml.reset();
  sbHtml.print(F(
    "<html>"
      "<head>"
        "<style>"
          "table, th, td { border: 1px solid grey; }"
          "th, td { padding: 5px; text-align: right; }"
          "body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }"
        "</style>"
      "</head>"
    "<body>"));
  sbHtml.print(F("<h2>Signal Attributes</h2>"));
  sbHtml.print(F("<p>Sampled over a "));
  sbHtml.print(lastStats.refreshTime);
  sbHtml.print(F("-second period</p>"));
  sbHtml.print(F("<table>"));
  sbHtml.print(rowStart);
  sbHtml.print(cellDiv);
  sbHtml.print(F("Total"));
  sbHtml.print(cellDiv);
  sbHtml.print(F("0 bit"));
  sbHtml.print(cellDiv);
  sbHtml.print(F("1 bit"));
  sbHtml.print(rowEnd);
  
  sbHtml.print(rowStart);
  sbHtml.print(F("DCC Bit Count"));
  sbHtml.print(cellDiv);
  sbHtml.print(lastStats.count/2);
  sbHtml.print(cellDiv);
  sbHtml.print(lastStats.count0/2);
  sbHtml.print(cellDiv);
  sbHtml.print(lastStats.count1/2);
  sbHtml.print(rowEnd);
  
  sbHtml.print(rowStart);
  sbHtml.print(F("Average Length (&micro;s)"));
  sbHtml.print(cellDiv);
  sbHtml.print(cellDiv);
  if (lastStats.count0 > 0) 
    sbHtml.print((float)lastStats.total0/lastStats.count0, 1);
  else 
    sbHtml.print(F("N/A"));
  sbHtml.print(cellDiv);
  if (lastStats.count1 > 0) 
    sbHtml.print((float)lastStats.total1/lastStats.count1, 1);
  else 
    sbHtml.print(F("N/A"));
  sbHtml.print(rowEnd);
  
  sbHtml.print(rowStart);
  sbHtml.print(F("Min-Max (&micro;s)"));
  sbHtml.print(cellDiv);
  sbHtml.print(cellDiv);
  if (lastStats.count0 > 0) {
    sbHtml.print(lastStats.min0);
    sbHtml.print('-');
    sbHtml.print(lastStats.max0);
  } else
    sbHtml.print(F("N/A"));
  sbHtml.print(cellDiv);
  if (lastStats.count1 > 0) {
    sbHtml.print(lastStats.min1);
    sbHtml.print('-');
    sbHtml.print(lastStats.max1);
  } else
    sbHtml.print(F("N/A"));
  sbHtml.print(rowEnd);
  
  sbHtml.print(rowStart);
  sbHtml.print(F("Max delta (&micro;s)"));
  sbHtml.print(cellDiv);
  sbHtml.print(cellDiv);
  if (lastStats.count0 > 0) {
    sbHtml.print(F("&lt;"));
    sbHtml.print(lastStats.max0BitDelta);
  } else
    sbHtml.print(F("N/A"));
  sbHtml.print(cellDiv);
  if (lastStats.count1 > 0) {
    sbHtml.print(F("&lt;"));
    sbHtml.print(lastStats.max1BitDelta);
  } else
    sbHtml.print(F("N/A"));
  sbHtml.print(rowEnd);
  
  sbHtml.print(rowStart);
  sbHtml.print(F("Glitches"));
  sbHtml.print(cellDiv);
  sbHtml.print(lastStats.glitchCount);
  sbHtml.print(rowEnd);

  sbHtml.print(rowStart);
  sbHtml.print(F("Valid Packets Received"));
  sbHtml.print(cellDiv);
  sbHtml.print(lastStats.packetCount);
  sbHtml.print(rowEnd);
  
  sbHtml.print(rowStart);
  sbHtml.print(F("Idle Packets Received"));
  sbHtml.print(cellDiv);
  sbHtml.print(lastStats.packetCount);
  sbHtml.print(rowEnd);

  sbHtml.print(rowStart);
  sbHtml.print(F("NMRA out of spec packets"));
  sbHtml.print(cellDiv);
  sbHtml.print(lastStats.outOfSpecRejectionCount);
  sbHtml.print(rowEnd);

  sbHtml.print(rowStart);
  sbHtml.print(F("RCN 5ms errors"));
  sbHtml.print(cellDiv);
  sbHtml.print(lastStats.rcn5msFailCount);
  sbHtml.print(rowEnd);

  sbHtml.print(rowStart);
  sbHtml.print(F("Checksum errors"));
  sbHtml.print(cellDiv);
  sbHtml.print(lastStats.checksumError);
  sbHtml.print(rowEnd);

  sbHtml.print(rowStart);
  sbHtml.print(F("Long packets"));
  sbHtml.print(cellDiv);
  sbHtml.print(lastStats.countLongPackets);
  sbHtml.print(rowEnd);

  sbHtml.print(F("</table>"));

  sbHtml.print(F("<h2>Half-Bit Count by Length</h2>"));
  sbHtml.print(F("<table>"));
  sbHtml.print(rowStart);
  sbHtml.print(F("Length (&micro;s)"));
  sbHtml.print(cellDiv);
  sbHtml.print(F("Count (1st Half)"));
  sbHtml.print(cellDiv);
  sbHtml.print(F("Count (2nd Half)"));
  sbHtml.print(rowEnd);

  for (int i=minBitLength; i<=maxBitLength; i++) {
    unsigned long c0 = lastStats.countByLength[0][i-minBitLength];
    unsigned long c1 = lastStats.countByLength[1][i-minBitLength];        
    if (c0 > 0 || c1 > 0) {
      sbHtml.print(rowStart);
      if (i == minBitLength) sbHtml.print(F("&le;"));
      else if (i == maxBitLength) sbHtml.print(F("&ge;"));
      sbHtml.print(i);
      sbHtml.print(cellDiv);
      sbHtml.print(c0);
      sbHtml.print(cellDiv);
      sbHtml.print(c1);
      sbHtml.print(rowEnd);
    }
  }
  sbHtml.print(F("</table>"));

  // Append the buffer of decoded packets (plain text).
  sbHtml.print(F("<h2>Decoded DCC Packets</h2>"));
  sbHtml.println(F("<pre>"));
  sbHtml.println(bufferPointer);
  sbHtml.println(F("</pre>"));
  sbHtml.println(F("</body></html>"));
}


// Declare singleton class instance.
HttpManagerClass HttpManager;

#endif
