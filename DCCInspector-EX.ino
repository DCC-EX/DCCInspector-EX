/* Copyright (c) 2021 Neil McKechnie
 * Parts based on DCC_Sniffer, Ruud Boer, October 2015
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
/* Boards:
 * - Arduino Nano Every * arduino:megaavr:nona4809:mode=off,CONSOLEBAUD=115200
 */

///////////////////////////////////////////////////////
//
// Add support for Arduino Nano Every with MegaCoreX: newHeiko Apr 2024
// Add buttons to web page for modifying options: NMcK Apr 2021
// Regroup functions into separate classes: NMcK Jan 2021
// Move configuration items to Config.h: NMcK Jan 2021
// Add OLED, and web server support on ESP32: NMcK Jan 2021
// Add capture for ESP32; refactor timers: NMcK Jan 2021
// Improve accuracy of timing and time calculations: NMcK Jan 2021
// Add support for compilation for ESP8266 and ESP32: NMcK Jan 2021
// Use ICR for pulse measurements, and display pulse length histogram: NMcK Dec 2020
// Add CPU load figures for the controller: NMcK December 2020
// Count, and optionally filter, edge noise from input: NMcK December 2020
// Display half-bit lengths rather than bit lengths: NMcK December 2020
// Improved comments; moved some strings to flash: NMcK November 2020
// Moved frame construction into interrupt code: NMcK July 2020
// Removed use of AVR timer interrupts: NMcK June 2020
// DCC packet analyze: Ruud Boer, October 2015
//
// The DCC signal is detected on Arduino digital pin 8 (ICP1 on Uno/Nano), or pin 49
// (ICP4 on Mega), or pin GPIO2 on ESP8266/ESP32, or any pin on Nano Every. This causes an
// interrupt, and in the interrupt response code the time between interrupts is measured.
//
// Use an opto-isolator between the track signal (~30V p-p) and the
// digital input (0 to 5V or 0 to 3.3V) to prevent burn-out.
//
// Written originally for Uno but also tested on Mega and Nano,
// should work on other architectures if simple options chosen (no USETIMER and
// no USE_DIO2).
//
// Also tested on ESP8266 (NodeMCU and Heltec Kit 8) and ESP32 (Heltec Kit 32).  Both of these
// support OLED display and HTTP server on WiFi.  See Config.h for configuration options.
// It will decode bits on the input pin which is GPIO2 for ESP8266 (labelled D4 on the NodeMCU) or 
// GPIO5 on the ESP32.
//
// The faster clock speeds on the ESP8266/ESP32 mean that interrupt jitter is less, but there is 
// still around +/- 4us evident on the ESP8266.  Also, the ESP8266 and ESP32, micros() does
// actually give a resolution of 1 microsecond (as opposed to 4us on the Arduino).  The ESP32
// gives the best performance and functionality, with its input capture capability alongside the 
// OLED/Wifi.
//
// The use of ICPn on the Arduino and ESP32 enables accurate timing to within 1us.  The millis() 
// function in the Arduino is no better than 10.5us or so accurate (6.5us interrupt jitter caused by the
// Arduino Timer0 interrupts, and 4us timer resolution) which is inadequate for diagnostic purposes.
//
// The selected digital input must have interrupt support, either via
// the level change interrupt (e.g. Arduino pin 2 or 3) or, preferably, input capture interrupt
// (e.g. pin 8 on Arduino Uno, pin 49 on Mega).  The ESP8266 does not have input capture functionality
// but any pin may be used for level change interrupt.
//
// The bit decoding is done by measuring the time between successive
// interrupts using the 'micros()' function so should be pretty portable.
// Optionally, it will use a faster 16-bit timer for higher resolution.  This is
// the default mode for Uno, Nano and Mega (USETIMER option).
//
// The counter SpareLoopCount is used to see how heavily loaded the processor is.
// With DCC interrupts off, it measures the count.  When the interrupts are attached
// it sees how much the count drops.  The percent load due to the interrupts is
// the percentage difference between the two figures.
// For example (my measurements on an Uno):
//    DCC off, SpareLoopCount=1043042 (over 4 secs)
//    DCC on, SpareLoopCount=768830 (over 4 secs)
//    CPU Load = (1043042-766830)/1043042*100 = 26%
//
// Loco address 3's speed command is optionally mapped onto a PWM output pin, allowing an 
// LED to be used to confirm that a controller is able to send recognisable 
// DCC commands.
//
// When outputting decoded DCC packets, duplicate loco speed packets and idle packets encountered
// within a time period will not be logged (as they are sent continuously).  However, all 
// accessory and loco function packets are displayed, even if repeated.
//
// Set the Serial Monitor Baud Rate to 115200 !!
//
// Keyboard commands that can be sent via Serial Monitor:
// 1 = 1s refresh time
// 2 = 2s 
// 3 = 4s (default)
// 4 = 8s
// 5 = 16s
// 6 = 4 DCC packet buffer
// 7 = 8
// 8 = 16
// 9 = 32 (default)
// 0 = 64
// a = show accessory packets toggle
// l = show locomotive packets toggle
// d = show diagnostics toggle
// h = show heartbeat toggle
// b = show half-bit counts by length toggle
// c = show cpu/irc usage in sniffer toggle
// f = input filter toggle
// s = set NMRA compliance strictness (0=none,1=decoder,2=controller)
// i = Display IP Address
// ? = help (show this information)
//
////////////////////////////////////////////////////////
#include <Arduino.h>

// Configurable parameter items now in separate include file.
#include "Config.h"

////////////////////////////////////////////////////////

#if defined(USE_DIO2) && (defined(ARDUINO_UNO_NANO) || defined(ARDUINO_MEGA))
#define GPIO_PREFER_SPEED
#include <DIO2.h>
#define digitalWrite(pin, state) digitalWrite2(pin, state)
#define digitalRead(pin) digitalRead2(pin)
#elif defined(USE_DIO2) && defined(ARDUINO_NANO_EVERY)
#define digitalWrite(pin, state) digitalWriteFast(pin, state)
#define digitalRead(pin) digitalReadFast(pin)
#define GPIO_PREFER_SPEED
#endif

#ifdef USETIMER
#include "EventTimer.h"
#else
#include "EventTimer_default.h"
#endif

// Add web server if required.
#if defined(USE_HTTPSERVER)
#include "HttpManager.h"
#endif

// Include OLED if required.
#if defined(USE_OLED)
#include "OledDisplay.h"
#endif

#include "StringBuilder.h"

// Statistics structures and functions.
#include "DCCStatistics.h"

const int nPackets = 16;  // Number of packet buffers
const int pktLength = 8;  // Max length+1 in bytes of DCC packet

// Variables shared by interrupt routine and main loop
volatile byte dccPacket[nPackets][pktLength];  // buffer to hold packets
volatile byte packetsPending = 0;              // Count of unprocessed packets
volatile byte activePacket =
    0;  // indicate which buffer is currently being filled
volatile bool filterInput =
    true;  // conditions input to remove transient changes
volatile byte strictMode =
    1;  // rejects frames containing out-of-spec bit lengths

// Variables used by main loop
byte packetHashListSize = 32;  // DCC packets checksum buffer size
bool showLoc = true;
bool showAcc = true;
bool showHeartBeat = true;
bool showDiagnostics = true;
bool showBitLengths = false;
bool showCpuStats = false;

byte inputPacket = 0;  // Index of next packet to be analysed in dccPacket array
byte pktByteCount = 0;
int packetHashListCounter = 0;
unsigned int packetHashList[64];
bool calibrated = false;
unsigned long lastRefresh = 0;
unsigned int inactivityCount = 0;

// Buffers for decoded packets, used by HTTP and OLED output.
#if defined(USE_HTTPSERVER)
char packetBuffer[5000] = "";
#elif defined(USE_OLED)
char packetBuffer[400] = "";
#else
char packetBuffer[1] = "";
#endif
StringBuilder sbPacketDecode(packetBuffer, sizeof(packetBuffer));

// Pre-declare capture function
bool INTERRUPT_SAFE capture(unsigned long halfBitLengthTicks);

//=======================================================================
// Perform the setup functions for the application.

void setup() {
  Serial.begin(SERIAL_SPEED);
  Serial.println(F("---"));
  Serial.println(F("DCC Packet Analyze initialising"));

#ifdef LEDPIN_ACTIVE
  pinMode(LEDPIN_ACTIVE, OUTPUT);
#endif
#ifdef LEDPIN_DECODING
  pinMode(LEDPIN_DECODING, OUTPUT);
#endif
#ifdef LEDPIN_DECODING
  pinMode(LEDPIN_DECODING, OUTPUT);
#endif
#ifdef LEDPIN_FAULT
  pinMode(LEDPIN_FAULT, OUTPUT);
#endif

  // Enable pullup in case there's no external pullup resistor.
  // External resistor is preferred as it can be lower, so
  // improve the switching speed of the Optocoupler.
  pinMode(INPUTPIN, INPUT_PULLUP);
  Serial.print("INPUTPIN=");
  Serial.println(INPUTPIN);

  if (!EventTimer.inputCaptureMode()) {
    // Output health warning...
    Serial.println(
        F("\r\n** WARNING Measurements will occasionally be out up to ~10us "
          "either way **"));
    Serial.println(
        F("**         because of inaccuracies in the micros() function.        "
          "    **"));
  }

#if defined(USE_OLED)
  // Start OLED display (if required).
  OledDisplay.begin(SDA_OLED, SCL_OLED);
#if defined(ARDUINO_HELTEC_WIFI_KIT_32)
  // Read battery voltage from pin GPIO37
  // The battery measurement is enabled via pin GPIO21
  digitalWrite(21, 0);
  analogSetWidth(12);  // 12 bits = 0-4095
  analogSetPinAttenuation(37, ADC_11db);
  adcAttachPin(37);

  uint32_t batValue = analogRead(37);
  // An input value of around 2600 is obtained for a
  // a measured battery voltage of 4100mV.  
  uint16_t batMV = batValue * 41 / 26;
  if (batMV < 3400) OledDisplay.append("Battery Low");
  digitalWrite(21, 1);  // Disable battery monitor

#endif
  OledDisplay.append("Initialising..");
#endif

// Start WiFi and HTTP server (if required).
#if defined(USE_HTTPSERVER)
  HttpManager.begin(WIFI_SSID, WIFI_PASSWORD, DNSNAME);
#endif

  // Set time for first output of statistics during calibration
  lastRefresh = millis();
  DCCStatistics.setRefreshTime(1);  // Finish calibrating after 1 second

  Serial.print(F("Calibrating... "));
}

//=======================================================================
// Main program loop.

void loop() {
  bool somethingDone = false;

  // The first bit runs one second after setup, and completes the
  // initialisation.
  if (!calibrated && millis() >= lastRefresh + 1000) {
    // Calibration cycle done, record the details.
    Serial.println(F("done."));
    calibrated = true;

    // Read (and discard) stats, then clear them.
    DCCStatistics.getAndClearStats();
    clearHashList();

    // Start recording data from DCC.
    if (!EventTimer.begin(INPUTPIN, capture)) {
      Serial.println(F("Unable to start EventTimer, check configured pin"));
      while (1)
        ;
    }

    DCCStatistics.setRefreshTime(4);
    Serial.print(F("Updates every "));
    Serial.print(DCCStatistics.getRefreshTime());
    Serial.println(F(" seconds"));
    Serial.println(F("---"));

    lastRefresh = millis();

  } else if (millis() >=
             lastRefresh +
                 (unsigned long)DCCStatistics.getRefreshTime() * 1000) {
    // The next part runs once every 'refresh time' seconds.  It primarily
    // captures, resets and
    //  outputs the statistics.

    if (showHeartBeat) Serial.println('-');

    // Snapshot and clear statistics
    Statistics stats = DCCStatistics.getAndClearStats();
    clearHashList();

    // Print DCC Statistics to the serial USB output.
    if (showDiagnostics) {
      DCCStatistics.writeFullStatistics(stats, showCpuStats, showBitLengths);
      Serial.println("--");
    }

// Output short version of DCC statistics to a buffer
// for use by OLED
#if defined(USE_OLED)
    OledDisplay.writeShortStatistics(stats);
    OledDisplay.append(sbPacketDecode.getString());  // Append decoded packets
    // Update OLED
    OledDisplay.refresh();
#endif

// Output full stats for HTTPServer to use
#if defined(USE_HTTPSERVER)
    HttpManager.setBuffer(sbPacketDecode.getString());
    HttpManager.writeHtmlStatistics(stats);
#endif

    sbPacketDecode.reset();  // Empty decoded packet list.

#if defined(ESP32)
    // Check if time to go to sleep on ESP32
    inactivityCount += DCCStatistics.getRefreshTime();
    if (inactivityCount > 120) {
// Go to sleep after 2 minutes of inactivity.
#if defined(USE_OLED)
      OledDisplay.reset();
      OledDisplay.append("Going to sleep..");
#endif
      Serial.println(F("*** Inactivity detected -- going to sleep ***"));
      delay(5000);
#if defined(ARDUINO_HELTEC_WIFI_KIT_32)
      // Turn off WiFi
      WiFi.disconnect(true);
      // Turn off Vext power to screen on Heltec Kit 32 V2
      pinMode(21, OUTPUT);
      digitalWrite(21, 1);
#endif
      esp_deep_sleep_start();
    }
#endif

    lastRefresh = millis();
    somethingDone = true;
  }

  // Check for DCC packets - if found, analyse and display them
  if (processDCC(Serial)) {
    somethingDone = true;
    inactivityCount = 0;
  }

  // Check for commands received over the USB serial connection.
  if (processCommands()) {
    somethingDone = true;
    inactivityCount = 0;
  }

#if defined(USE_OLED)
  OledDisplay.checkButton();
#endif

  // Increment CPU loop counter.  This is done if nothing else was.
  // If the counter never gets incremented, it means that the
  // CPU is fully loaded doing other things and has no spare time.
  if (!somethingDone) DCCStatistics.updateLoopCount();

#if defined(USE_HTTPSERVER)
  HttpManager.process();
#endif

  UpdateLED();
}

//=======================================================================
// Function invoked (from interrupt handler) on change of state of INPUTPIN.
//  It measures the time between successive changes (half-cycle of DCC
//  signal).  Depending on the value, it decodes 0 or a 1 for alternate
//  half-cycles.  A 0 half-bit is nominally 100us per half-cycle (NMRA says
//  90-10000us) and a 1 half-bit is nominally 58us (52-64us).  We treat a
//  half-bit duration < 80us as a '1' half-bit, and a duration >= 80us as a '0'
//  half-bit. Prologue and framing bits are detected and stripped, and data
//  bytes are then stored in the packet queue for processing by the main loop.
//
bool INTERRUPT_SAFE capture(unsigned long halfBitLengthTicks) {
  static byte preambleOneCount = 0;
  static boolean preambleFound = false;
  static int newByte =
      0;  // Accumulator for input bits until complete byte found.
  static int inputBitCount = 0;  // Number of bits read in current newByte.
  static int inputByteNumber =
      0;  // Number of bytes read into active dccPacket buffer so far
  static byte interruptCount = 0;
  static byte previousBitValue = 0, previousDiginState = 0;
  static unsigned int previousHalfBitLengthTicks = 0;
  static byte altbit = 0;  // 0 for first half-bit and 1 for second.
  byte bitValue;

  // The most critical parts are done first - read state of digital input.
  byte diginState = digitalRead(INPUTPIN);

  // Set a high bound on the half bit length
  if (halfBitLengthTicks > 1200 * TICKSPERMICROSEC)
    halfBitLengthTicks = 1200 * TICKSPERMICROSEC;  // microseconds.

  // Calculate time between interrupts in microseconds.
  unsigned int interruptInterval = halfBitLengthTicks / TICKSPERMICROSEC;

  // Precondition input?
  if (filterInput) {
    // Check that the digital input has actually changed since last interrupt,
    // and that the gap between interrupts is realistic.
    if (interruptCount > 0 &&
        (diginState == previousDiginState || interruptInterval <= 3)) {
      // No change in digital, or it was fleeting.  Ignore.
      DCCStatistics.recordGlitch();
      return false;  // reject interrupt
    }
  }

  // If we get here, the interrupt looks valid, i.e. the digital input really
  // did change its state more than 3us after its last change. Calculate
  // difference between current bit half and preceding one, rounding up to next
  // microsecond. This will only be recorded on alternate half-bits, i.e. where
  // the previous and current half-bit make a complete bit.
  long deltaTicks = halfBitLengthTicks - previousHalfBitLengthTicks;
  if (deltaTicks < 0) deltaTicks = -deltaTicks;
  unsigned int delta = (deltaTicks + TICKSPERMICROSEC - 1) / TICKSPERMICROSEC;

  // Check length of half-bit
  if (interruptInterval < 80)
    bitValue = 1;
  else
    bitValue = 0;

  // Record input state and timer values ready for next interrupt
  previousDiginState = diginState;
  previousHalfBitLengthTicks = halfBitLengthTicks;

  // If first or second interrupt, then exit as the previous state is
  // incomplete.
  if (interruptCount < 2) {
    interruptCount++;
    previousBitValue = bitValue;
    return true;
  }

#ifdef LEDPIN_ACTIVE
  digitalWrite(LEDPIN_ACTIVE, 1);
#endif

  // Check if we're on the first or second half of the bit.
  if (bitValue != previousBitValue) {
    // First half of new bit received
    altbit = false;
  } else {
    // Toggle for alternate half-bits
    altbit = !altbit;
  }
  previousBitValue = bitValue;

  // Update statistics
  DCCStatistics.recordHalfBit(altbit, bitValue, interruptInterval, delta);

  // Store interrupt interval for use on next interrupt.
  previousHalfBitLengthTicks = halfBitLengthTicks;

  // If this is the second half-bit then we've got a whole bit!!
  if (altbit) {
    bool rejectBit = false;
    if (strictMode == 2) {
      // Validate bit lengths against NMRA spec for controllers
      if (bitValue == 0) {
        if (interruptInterval < 95 || interruptInterval > 9900) {
          rejectBit = true;
        }
      } else {
        if (interruptInterval < 55 || interruptInterval > 61 || delta > 3) {
          rejectBit = true;
        }
      }
    } else if (strictMode == 1) {
      // Validate bit lengths against NMRA spec for decoders.
      if (bitValue == 0) {
        if (interruptInterval < 90 || interruptInterval > 10000) {
          rejectBit = true;
        }
      } else {
        if (interruptInterval < 52 || interruptInterval > 64 || delta > 6) {
          rejectBit = true;
        }
      }
    }
    // Record error only if we're in a packet (preamble has been read).
    if (rejectBit && preambleFound) {
      DCCStatistics.recordOutOfSpecRejection();
      // Search for next packet
      preambleFound = 0;
      preambleOneCount = 0;
    }

    // Now we've got a bit, process it.  The message comprises the following:
    //   Preamble: 10 or more '1' bits followed by a '0' start bit.
    //   Groups of 9 bits each containing data byte of 8 bits, followed by a
    //   '0' bit (if message not yet finished), or a '1' bit (if the byte is
    //   the last byte of the message, i.e. the checksum).
    //
    if (!preambleFound) {
      if (bitValue == 1) {
        // Reading preamble perhaps...
        preambleOneCount++;
      } else if (preambleOneCount < 10) {  // and bitValue==0)
        // Preamble not yet found, but zero bit encountered.  Restart preable
        // count.
        preambleOneCount = 0;
      } else {  // preambleOneCount >= 10 and bitValue==0
        // Start bit found at end of preamble, so prepare to process data.
        preambleFound = true;
        newByte = 0;
        inputBitCount = 0;
        inputByteNumber = 0;
      }
    } else {  // Preamble previously found, so this is a message bit
      if (packetsPending == nPackets) {
        // Previous DCC packets haven't been processed by the main loop,
        // so there is no buffer for the incoming message.
        // Discard incoming message and scan for another preamble.
        preambleFound = false;
        preambleOneCount = 0;

        // Record this event in a counter.
        DCCStatistics.recordLostPacket();

      } else {
        // Preamble read, packet buffer available, so message bit can be stored!
        if (inputBitCount == 8) {  // Byte previously completed, so this bit is
                                   // the interbyte marker
          if (bitValue == 0) {  // Interbyte marker is zero, so prepare for next
                                // byte of data
            inputBitCount = 0;
          } else {  // one-bit found, marks end of packet
            // End of packet found
            dccPacket[activePacket][0] =
                inputByteNumber;  // save number of bytes
            packetsPending++;     // flag that packet is ready for processing
            if (++activePacket >= nPackets)
              activePacket = 0;     // move to next packet buffer
            preambleFound = false;  // scan for another preamble
            preambleOneCount =
                1;  // allow the current bit to be counted in the preamble.
          }
        } else {  // Reading packet data at this point.
          // Append received bit to the current new byte.
          newByte = (newByte << 1) | bitValue;
          if (++inputBitCount == 8) {  // Completed byte, save byte (if room)
            if (inputByteNumber < pktLength - 1)
              dccPacket[activePacket][++inputByteNumber] = newByte;
            else {  // packet has filled buffer so no more bits can be stored!
              packetsPending++;  // flag that packet is ready for processing
              if (++activePacket >= nPackets)
                activePacket = 0;     // move to next packet buffer
              preambleFound = false;  // scan for another preamble
              preambleOneCount = 0;
              // Record this event in a counter.
              DCCStatistics.recordLongPacket();
            }
            newByte = 0;
          }
        }
      }
    }
  }

#ifdef LEDPIN_ACTIVE
  // Turn out ACTIVE LED.
  digitalWrite(LEDPIN_ACTIVE, 0);
#endif

  // Calculate time taken in interrupt code between the measured time of event
  // to POINTB.

  unsigned int interruptDuration =
      EventTimer.elapsedTicksSinceLastEvent() / TICKSPERMICROSEC;  // POINTB

  // Assume that there are about 25 cycles of instructions in this function that
  // are not measured, and that the prologue in dispatching the function (saving
  // registers etc) is around 51 cycles and the epilogue (restoring registers
  // etc) is around 35 cycles.  This adds a further (51+25+35)/16MHz=6.9us to
  // the calculation. See
  // https://billgrundmann.wordpress.com/2009/03/02/the-overhead-of-arduino-interrupts/.
  // However, if the Input Capture mode is used, then this will be much smaller.
  // So ignore it.
  // interruptDuration += 7;

  // Record result
  DCCStatistics.recordInterruptHandlerTime(interruptDuration);

  return true;  // Accept interrupt.
}

//=======================================================================
// Connect the scan routine to the interrupt.  It will execute on
// all changes (0->1 and 1->0).

void beginBitDetection() { EventTimer.begin(INPUTPIN, capture); }

//=======================================================================
// PrintPacketBits prints the raw DCC packet contents to the
// nominated Print stream (e.g. Serial).

void printPacketBits(Print &output, int index) {
  output.print(' ');
  for (byte i = 1; i < dccPacket[index][0]; i++) {
    output.print(' ');
    byte b = dccPacket[index][i];
    for (int bit = 0; bit < 8; bit++) {
      output.print(b & 0x80 ? '1' : '0');
      b <<= 1;
    }
  }
}

//=======================================================================
// ClearDCCData clears the contents of the packetHashList array and resets
// the statistics.
// The packetHashList array normally contains the checksums of received
// DCC packets, and is used to suppress the decoding of repeated packets.

void clearHashList() {
  for (byte n = 0; n < packetHashListSize; n++) packetHashList[n] = 0;
  packetHashListCounter = 0;
}

//=======================================================================
// UpdateLED is called in the main loop to set/reset the LED fault indication
// in the event of a fault being detected within the sample period.

void UpdateLED() {
#ifdef LEDPIN_FAULT
  static bool ledLit = false;
  if (DCCStatistics.faultPresent()) {
    if (!ledLit) {
      digitalWrite(LEDPIN_FAULT, 1);
      ledLit = true;
    }
  } else {
    if (ledLit) {
      digitalWrite(LEDPIN_FAULT, 0);
      ledLit = false;
    }
  }
#endif
}

//=======================================================================
// Validate received packet and pass to decoder.
// Return false if nothing done.

bool processDCC(Print &output) {
  byte isDifferentPacket = 0;

  if (!packetsPending) {
    return false;
  }

  pktByteCount = dccPacket[inputPacket][0];
  // Check packet isn't empty
  if (pktByteCount > 0) {
    // Calculate and verify checksum
    byte checksum = 0;
    for (byte n = 1; n <= pktByteCount; n++)
      checksum ^= dccPacket[inputPacket][n];
    if (checksum) {  // Result should be zero, if not it's an error!
      DCCStatistics.recordChecksumError();
    } else {
// There is a new packet with a correct checksum
#ifdef LEDPIN_DECODING
      digitalWrite(LEDPIN_DECODING, 1);
#endif

      // Hooray - we've got a packet to decode, with no errors!
      DCCStatistics.recordPacket();

      // Generate a cyclic hash based on the packet contents for checking if
      // we've seen a similar packet before.
      isDifferentPacket = true;
      unsigned int hash =
          dccPacket[inputPacket][pktByteCount];  // calculate checksum
      for (byte n = 1; n < pktByteCount; n++)
        hash = ((hash << 5) | (hash >> 11)) ^ dccPacket[inputPacket][n];

      // Check if packet's checksum is already in the list.
      for (byte n = 0; n < packetHashListSize; n++) {
        if (hash == packetHashList[n]) isDifferentPacket = false;
      }

      if (isDifferentPacket) {
        packetHashList[packetHashListCounter++] =
            hash;  // add new packet's hash to the list
        if (packetHashListCounter >= packetHashListSize)
          packetHashListCounter = 0;

        DecodePacket(output, inputPacket, isDifferentPacket);
      }

// Optional test led whose brightness depends on loco speed setting.
#ifdef LEDPIN_LOCOSPEED
      // Output to LED
      if (dccPacket[inputPacket][1] == 0B00000011 &&
          dccPacket[inputPacket][2] == 0B00111111) {
        analogWrite(LEDPIN_LOCOSPEED,
                    map(dccPacket[inputPacket][3] & 0B01111111, 0, 127, 0, 255));
      }
#endif

#ifdef LEDPIN_DECODING
      digitalWrite(LEDPIN_DECODING, 0);
#endif
    }
  }
  packetsPending--;  // Free packet buffer.
  if (++inputPacket >= nPackets) inputPacket = 0;

  return true;
}

//=======================================================================
// Read data from the dccPacket structure and decode into
// textual representation.  Send results out over the USB serial
// connection.

void DecodePacket(Print &output, int inputPacket, bool isDifferentPacket) {
  byte instrByte1;
  byte decoderType;  // 0=Loc, 1=Acc
  unsigned int decoderAddress;
  byte speed;
  bool outputDecodedData = false;

  char tempBuffer[100];
  StringBuilder sbTemp(tempBuffer, sizeof(tempBuffer));

  // First determine the decoder type and address.
  if (dccPacket[inputPacket][1] == 0B11111111) {  // Idle packet
    if (isDifferentPacket) {
      sbTemp.print(F("Idle "));
      outputDecodedData = true;
    }
    decoderType = 255;
  } else if (!bitRead(dccPacket[inputPacket][1],
                      7)) {  // bit7=0 -> Loc Decoder Short Address
    decoderAddress = dccPacket[inputPacket][1];
    instrByte1 = dccPacket[inputPacket][2];
    decoderType = 0;
  } else {
    if (bitRead(dccPacket[inputPacket][1],
                6)) {  // bit7=1 AND bit6=1 -> Loc Decoder Long Address
      decoderAddress = 256 * (dccPacket[inputPacket][1] & 0B00111111) +
                       dccPacket[inputPacket][2];
      instrByte1 = dccPacket[inputPacket][3];
      decoderType = 0;
    } else {  // bit7=1 AND bit6=0 -> Accessory Decoder
      decoderAddress = dccPacket[inputPacket][1] & 0B00111111;
      instrByte1 = dccPacket[inputPacket][2];
      decoderType = 1;
    }
  }

  // Handle decoder type 0 and 1 separately.
  if (decoderType == 1) {  // Accessory Basic
    if (showAcc) {
      if (instrByte1 & 0B10000000) {  // Basic Accessory
        decoderAddress = (((~instrByte1) & 0B01110000) << 2) + decoderAddress;
        byte port = (instrByte1 & 0B00000110) >> 1;
        sbTemp.print(F("Acc "));
        sbTemp.print((decoderAddress - 1) * 4 + port + 1);
        sbTemp.print(' ');
        sbTemp.print(decoderAddress);
        sbTemp.print(F(":"));
        sbTemp.print(port);
        sbTemp.print(' ');
        sbTemp.print(bitRead(instrByte1, 3));
        if (bitRead(instrByte1, 0))
          sbTemp.print(F(" On"));
        else
          sbTemp.print(F(" Off"));
      } else {  // Accessory Extended NMRA spec is not clear about address and
                // instruction format !!!
        sbTemp.print(F("Acc Ext "));
        decoderAddress = (decoderAddress << 5) +
                         ((instrByte1 & 0B01110000) >> 2) +
                         ((instrByte1 & 0B00000110) >> 1);
        sbTemp.print(decoderAddress);
        sbTemp.print(F(" Asp "));
        sbTemp.print(dccPacket[inputPacket][3], BIN);
      }
      outputDecodedData = true;
    }
  } else if (decoderType == 0) {  // Loco / Multi Function Decoder
    if (showLoc && isDifferentPacket) {
      sbTemp.print(F("Loc "));
      sbTemp.print(decoderAddress);
      byte instructionType = instrByte1 >> 5;
      byte value;
      switch (instructionType) {
        case 0:
          sbTemp.print(F(" Control"));
          break;

        case 1:                           // Advanced Operations
          if (instrByte1 == 0B00111111) {  // 128 speed steps
            if (bitRead(dccPacket[inputPacket][pktByteCount - 1], 7))
              sbTemp.print(F(" Fwd128 "));
            else
              sbTemp.print(F(" Rev128 "));
            byte speed = dccPacket[inputPacket][pktByteCount - 1] & 0B01111111;
            if (!speed)
              sbTemp.print(F("Stop"));
            else if (speed == 1)
              sbTemp.print(F("Estop"));
            else
              sbTemp.print(speed - 1);
          } else if (instrByte1 == 0B00111110) {  // Speed Restriction
            if (bitRead(dccPacket[inputPacket][pktByteCount - 1], 7))
              sbTemp.print(F(" On "));
            else
              sbTemp.print(F(" Off "));
            sbTemp.print(dccPacket[inputPacket][pktByteCount - 1] & 0B01111111);
          }
          break;

        case 2:  // Reverse speed step
          speed = ((instrByte1 & 0B00001111) << 1) - 3 + bitRead(instrByte1, 4);
          if (speed == 253 || speed == 254)
            sbTemp.print(F(" Stop"));
          else if (speed == 255 || speed == 0)
            sbTemp.print(F(" EStop"));
          else {
            sbTemp.print(F(" Rev28 "));
            sbTemp.print(speed);
          }
          break;

        case 3:  // Forward speed step
          speed = ((instrByte1 & 0B00001111) << 1) - 3 + bitRead(instrByte1, 4);
          if (speed == 253 || speed == 254)
            sbTemp.print(F(" Stop"));
          else if (speed == 255 || speed == 0)
            sbTemp.print(F(" EStop"));
          else {
            sbTemp.print(F(" Fwd28 "));
            sbTemp.print(speed);
          }
          break;

        case 4:  // Loc Function L-4-3-2-1
          sbTemp.print(F(" L F4-F1 "));
          sbTemp.print(instrByte1 & 0B00011111, BIN);
          break;

        case 5:  // Loc Function 8-7-6-5
          if (bitRead(instrByte1, 4)) {
            sbTemp.print(F(" F8-F5 "));
            sbTemp.print(instrByte1 & 0B00001111, BIN);
          } else {  // Loc Function 12-11-10-9
            sbTemp.print(F(" F12-F9 "));
            sbTemp.print(instrByte1 & 0B00001111, BIN);
          }
          break;

        case 6:  // Future Expansions
          switch (instrByte1 & 0B00011111) {
            case 0:  // Binary State Control Instruction long form
              sbTemp.print(F(" BinSLong "));
              sbTemp.print(
                  128 * ((uint16_t)dccPacket[inputPacket][pktByteCount - 1]) +
                  (dccPacket[inputPacket][pktByteCount - 2] & 127));
              if bitRead (dccPacket[inputPacket][pktByteCount - 2], 7)
                sbTemp.print(F(" On"));
              else
                sbTemp.print(F(" Off"));
              break;
            case 0B00011101:  // Binary State Control
              sbTemp.print(F(" BinShort "));
              sbTemp.print(dccPacket[inputPacket][pktByteCount - 1] &
                           0B01111111);
              if bitRead (dccPacket[inputPacket][pktByteCount - 1], 7)
                sbTemp.print(F(" On"));
              else
                sbTemp.print(F(" Off"));
              break;
            case 0B00011110:  // F13-F20 Function Control
              sbTemp.print(F(" F20-F13 "));
              sbTemp.print(dccPacket[inputPacket][pktByteCount - 1], BIN);
              break;
            case 0B00011111:  // F21-F28 Function Control
              sbTemp.print(F(" F28-F21 "));
              sbTemp.print(dccPacket[inputPacket][pktByteCount - 1], BIN);
              break;
            case 0B00011000:  // F29-F36 Function Control
              sbTemp.print(F(" F36-F29 "));
              sbTemp.print(dccPacket[inputPacket][pktByteCount - 1], BIN);
              break;
            case 0B00011001:  // F37-F44 Function Control
              sbTemp.print(F(" F44-F37 "));
              sbTemp.print(dccPacket[inputPacket][pktByteCount - 1], BIN);
              break;
            case 0B00011010:  // F45-F52 Function Control
              sbTemp.print(F(" F52-F45 "));
              sbTemp.print(dccPacket[inputPacket][pktByteCount - 1], BIN);
              break;
            case 0B00011011:  // F53-F60 Function Control
              sbTemp.print(F(" F60-F53 "));
              sbTemp.print(dccPacket[inputPacket][pktByteCount - 1], BIN);
              break;
            case 0B00011100:  // F61-F68 Function Control
              sbTemp.print(F(" F68-F61 "));
              sbTemp.print(dccPacket[inputPacket][pktByteCount - 1], BIN);
              break;
            default:
              sbTemp.print(F(" Unknown"));
              break;
          }
          break;

        case 7:
          sbTemp.print(F(" CV "));
          value = dccPacket[inputPacket][pktByteCount - 1];
          if (instrByte1 & 0B00010000) {  // CV Short Form
            byte cvType = instrByte1 & 0B00001111;
            switch (cvType) {
              case 0B00000010:
                sbTemp.print(F("23 "));
                sbTemp.print(value);
                break;
              case 0B00000011:
                sbTemp.print(F("24 "));
                sbTemp.print(value);
                break;
              case 0B00001001:
                sbTemp.print(F("Lock "));
                sbTemp.print(value);
                break;
              default:
                sbTemp.print(F("Unknown"));
                sbTemp.print(' ');
                sbTemp.print(value);
                break;
            }
          } else {  // CV Long Form
            int cvAddress = 256 * (instrByte1 & 0B00000011) +
                            dccPacket[inputPacket][pktByteCount - 2] + 1;
            sbTemp.print(cvAddress);
            sbTemp.print(' ');
            switch (instrByte1 & 0B00001100) {
              case 0B00000100:  // Verify Byte
                sbTemp.print(F("Verify "));
                sbTemp.print(value);
                break;
              case 0B00001100:  // Write Byte
                sbTemp.print(F("Write "));
                sbTemp.print(value);
                break;
              case 0B00001000:  // Bit Write
                sbTemp.print(F("Bit "));
                if (value & 0B00010000)
                  sbTemp.print(F("Vrfy "));
                else
                  sbTemp.print(F("Wrt "));
                sbTemp.print(value & 0B00000111);
                sbTemp.print(' ');
                sbTemp.print((value & 0B00001000) >> 3);
                break;
              default:
                sbTemp.print(F("Unknown"));
                break;
            }
          }
          break;

        default:
          sbTemp.print(F(" Unknown"));
          break;
      }
      outputDecodedData = true;
    }
  }

  if (outputDecodedData) {
// If not using HTTP, append decoded packet to packet buffer
//  as-is (without binary dump).  It's all that fits on OLED.
#if !defined(USE_HTTPSERVER)
    sbPacketDecode.println(sbTemp.getString());
    sbPacketDecode.end();
#endif

    // Append binary dump for Serial and HTTP
    sbTemp.setPos(21);
    sbTemp.print(' ');
    printPacketBits(sbTemp, inputPacket);
    sbTemp.end();  // terminate string in buffer.

    // Get reference to buffer containing results.
    char *decodedPacket = sbTemp.getString();

#if defined(USE_HTTPSERVER)
    // Append decoded packet data and binary dump to string buffer.
    sbPacketDecode.println(decodedPacket);
    sbPacketDecode.end();
#endif

    // Also print to USB serial, and dump packet in hex.
    output.println(decodedPacket);
  }
}

//=======================================================================
// Process commands sent over the USB serial connection.
//  Return false if nothing done.

bool processCommands() {
  if (Serial.available()) {
    switch (Serial.read()) {
      case 49:
        Serial.println(F("Refresh Time = 1s"));
        DCCStatistics.setRefreshTime(1);
        break;
      case 50:
        Serial.println(F("Refresh Time = 2s"));
        DCCStatistics.setRefreshTime(2);
        break;
      case 51:
        Serial.println(F("Refresh Time = 4s"));
        DCCStatistics.setRefreshTime(4);
        break;
      case 52:
        Serial.println(F("Refresh Time = 8s"));
        DCCStatistics.setRefreshTime(8);
        break;
      case 53:
        Serial.println(F("Refresh Time = 16s"));
        DCCStatistics.setRefreshTime(16);
        break;
      case 54:
        Serial.println(F("Buffer Size = 4"));
        packetHashListSize = 2;
        break;
      case 55:
        Serial.println(F("Buffer Size = 8"));
        packetHashListSize = 8;
        break;
      case 56:
        Serial.println(F("Buffer Size = 16"));
        packetHashListSize = 16;
        break;
      case 57:
        Serial.println(F("Buffer Size = 32"));
        packetHashListSize = 32;
        break;
      case 48:
        Serial.println(F("Buffer Size = 64"));
        packetHashListSize = 64;
        break;
      case 'a':
      case 'A':
        showAcc = !showAcc;
        Serial.print(F("show accessory packets = "));
        Serial.println(showAcc);
        break;
      case 'l':
      case 'L':
        showLoc = !showLoc;
        Serial.print(F("show loco packets = "));
        Serial.println(showLoc);
        break;
      case 'h':
      case 'H':
        showHeartBeat = !showHeartBeat;
        Serial.print(F("show heartbeat = "));
        Serial.println(showHeartBeat);
        break;
      case 'd':
      case 'D':
        showDiagnostics = !showDiagnostics;
        Serial.print(F("show diagnostics = "));
        Serial.println(showDiagnostics);
        break;
      case 'f':
      case 'F':
        filterInput = !filterInput;
        Serial.print(F("filter input = "));
        Serial.println(filterInput);
        break;
      case 's':
      case 'S':
        strictMode = (strictMode + 1) % 3;
        Serial.print(F("NMRA validation level = "));
        Serial.println(strictMode);
        break;
      case 'b':
      case 'B':
        showBitLengths = !showBitLengths;
        Serial.print(F("show bit lengths = "));
        Serial.println(showBitLengths);
        break;
      case 'c':
      case 'C':
        showCpuStats = !showCpuStats;
        Serial.print(F("show Cpu stats = "));
        Serial.println(showCpuStats);
        break;
      case 'i':
#if defined(USE_HTTPSERVER)
      case 'I':
        HttpManager.getIP();
        break;
#endif
      case '?':
        Serial.println();
        Serial.println(
            F("Keyboard commands that can be sent via Serial Monitor:"));
        Serial.println(F("1 = 1s refresh time"));
        Serial.println(F("2 = 2s"));
        Serial.println(F("3 = 4s (default)"));
        Serial.println(F("4 = 8s"));
        Serial.println(F("5 = 16s"));
        Serial.println(F("6 = 4 DCC packet buffer"));
        Serial.println(F("7 = 8"));
        Serial.println(F("8 = 16"));
        Serial.println(F("9 = 32 (default)"));
        Serial.println(F("0 = 64"));
        Serial.println(F("a = show accessory packets toggle"));
        Serial.println(F("l = show locomotive packets toggle"));
        Serial.println(F("d = show diagnostics toggle"));
        Serial.println(F("h = show heartbeat toggle"));
        Serial.println(F("b = show half-bit counts by length toggle"));
        Serial.println(F("c = show cpu/irc usage in sniffer"));
        Serial.println(F("f = input filter toggle"));
        Serial.println(
            F("s = set NMRA compliance strictness "
              "(0=none,1=decoder,2=controller)"));
#if defined(USE_HTTPSERVER)
        Serial.println(F("i = show IP Address if connected"));
#endif
        Serial.println(F("? = help (show this information)"));
        Serial.print(F("ShowLoco "));
        Serial.print(showLoc);
        Serial.print(F(" / ShowAcc "));
        Serial.print(showAcc);
        Serial.print(F(" / RefreshTime "));
        Serial.print(DCCStatistics.getRefreshTime());
        Serial.print(F("s / BufferSize "));
        Serial.print(packetHashListSize);
        Serial.print(F(" / Filter "));
        Serial.print(filterInput);
        Serial.print(F(" / Strict Bit Validation "));
        Serial.println(strictMode);
        Serial.println();
        break;
    }
    return true;
  } else
    return false;
}
