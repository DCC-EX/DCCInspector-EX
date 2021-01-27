#ifndef config_h
#define config_h

// The following logic should work for all supported platforms, hopefully.
#if defined(ESP_PLATFORM)
  #if !defined(ESP32)
    #define ESP32
  #endif
#elif defined(ESP8266)
// nothing needs done here.
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
  #define ARDUINO_UNO_NANO
#elif defined(ARDUINO_AVR_MEGA2560)
  #define ARDUINO_MEGA
#else
  #error "Platform not recognised"
#endif

// Uncomment the "#define USE_DIO2" to use the 'Fast digital i/o library' DIO2
// in place of the normal digitalRead() and digitalWrite().
// Reduces CPU load by about 17%.  Only applicable to Arduino Uno/Mega/Nano.
#define USE_DIO2

// Uncomment the "#define USETIMER" to perform the timing using the ATmega328's 
// and ESP32's Input Capture mode which captures the time through hardware instead
// of software.  This enables a higher accuracy and consistency, and because we can capture the 
// Timer counter register (TCNTn) at the time of the digital change, it is immune to 
// timing errors caused by other interrupts.
// Works on Arduino Uno (Timer1/pin D8)
//          Arduino Nano (Timer1/pin D8)
//          Arduino Mega (Timer4/pin D49)
// If we don't use this, then the selected input pin must support change interrupt 
// (defaults to pin D2 on Uno, Nano and Mega, GPIO2 on ESP8266 and GPIO5 on ESP32.
#define USETIMER 

// Input pin definitions.  Defaults:
//    Nano (USETIMER):               8
//    Uno (USETIMER):                8
//    Mega (USETIMER):               49
//    Nano/Uno/Mega, Non-USETIMER:   2
//    ESP32:                         5
//    ESP8266:                       2
//    Other:                         2
#if defined(USETIMER)
  #if defined(ARDUINO_UNO_NANO)
    #define INPUTPIN 8
  #elif defined(ARDUINO_MEGA)
    #define INPUTPIN 49
  #elif defined(ESP32)
    #define INPUTPIN 5
  #else
    // Assume timer not supported, use default pin
    #undef USETIMER
    #define INPUTPIN 2
  #endif
#else
  #if defined(ESP32) 
    #define INPUTPIN 5
  #else
    #define INPUTPIN 2
  #endif
#endif

// Uncomment following lines to enable OLED output on pins SDA_OLED and SCL_OLED.
//  (ESP or Mega only).
#define USE_OLED        

#if defined(ESP8266) // Heltec Kit 8 has pins 4/15 for I2C.
  #define SDA_OLED 4   
  #define SCL_OLED 5
  #define OLED_RESET 16
  #define OLED_I2CADDRESS 0x3C
  #define SCREEN_WIDTH 128
  #define SCREEN_HEIGHT 32
#elif defined(ESP32)    // Heltec Kit 32 has pins 4/15 predefined for I2C.
  // #define SDA_OLED 4
  // #define SCL_OLED 15
  #define OLED_RESET 16
  #define OLED_I2CADDRESS 0x3C
  #define SCREEN_WIDTH 128
  #define SCREEN_HEIGHT 64
#else                   // Other boards - edit as appropriate.
  #define SDA_OLED 4
  #define SCL_OLED 5
  #define OLED_RESET 16
  #define OLED_I2CADDRESS 0x3D
  #define SCREEN_WIDTH 128
  #define SCREEN_HEIGHT 64
#endif
// Button for selecting new page on OLED.  The button action is carried out
// when the pin goes from 1 to 0 (pull-up is enabled).
#define BUTTONPIN 0

// LED pin definitions - uncomment and assign as required.
//#define LEDPIN_ACTIVE 13    // Shows interrupts being received, ie DCC active
//#define LEDPIN_LOCOSPEED 3  // Driven from loco speed packet for loco 3
//#define LEDPIN_DECODING 7   // lights when a packet with valid checksum is received
//#define LEDPIN_FAULT 6      // Lights when a checksum error or glitch is encountered.

// Uncomment the following line to enable HTTP Server (ESP32 or ESP8266 only).
#define USE_HTTPSERVER

// SSID and password can be configured here.  However, the server will, by preference,
// connect using the same credentials as last time; if that fails it will try WPS; 
// only if that fails too, will it use the credentials below.
#define WIFI_SSID ""
#define WIFI_PASSWORD ""

// Name used by mDNS to register the device.  On some browsers it can be accessed
// through this name with a ".local" suffix (e.g. http://DccInspector.local/).
#define DNSNAME "DccInspector"

// OLED isn't supported on Uno or Nano
#if defined(ARDUINO_UNO_NANO)
  #if defined(USE_OLED)
    #undef USE_OLED
  #endif
#endif

// HTTP Server isn't supported on Uno or Nano or Mega
#if defined(ARDUINO_UNO_NANO) | defined(ARDUINO_MEGA)
  #if defined(USE_HTTPSERVER)
    #undef USE_HTTPSERVER
  #endif
#endif

#define SERIAL_SPEED 115200

#if defined(ESP32) || defined(ESP8266) || defined(ESP_PLATFORM)
  #define INTERRUPT_SAFE IRAM_ATTR
#else
  #define INTERRUPT_SAFE
#endif

#endif
