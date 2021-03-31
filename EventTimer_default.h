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
 * EventTimer_default.h
 * 
 *  Timer functions for measuring elapsed time between events.  In this version,
 *  the functions use the micros() function available on all Arduino controllers
 *  as well as ESP8266, ESP32 etc.
 *  
 *  This module acts as the fallback if no specific support is available e.g. 
 *  through an Input Capture Register.
 */

#ifndef eventtimer_default_h
#define eventtimer_default_h

#include <Arduino.h>

#define TICKSPERMICROSEC 1

// ESP platform must have IRAM_ATTR on any function called from
//  interrupt handlers, so that they aren't executed from flash.
#if defined(ESP32) || defined(ESP8266) || defined(ESP_PLATFORM)
  #define INTERRUPT_SAFE IRAM_ATTR
#else
  #define INTERRUPT_SAFE
#endif

// Predeclare event handler function for later...
void eventHandler();

/* 
 *  User event handler is sent the time since the last valid event.
 *  If the user event handler decides that this event isn't valid 
 *  (e.g. time since last one is too short) then it should return 
 *  false to reject the event.  If the event is valid, then it 
 *  should return true to accept it.
 */
typedef bool EventHandler(unsigned long eventSpacing);

class EventTimerClass {
public:

  // Initialise the object instance, validating that the input pin is
  //  correct and noting the reference to the user handler for future use.
  bool begin(int pin, EventHandler userHandler) {
    this->pin = pin;
    int interruptNumber = digitalPinToInterrupt(pin);
    if (interruptNumber < 0) {
      Serial.print("ERROR: Pin "); Serial.print(pin); Serial.println(" has no interrupt support");
      return false;
    }
    this->callUserHandler = userHandler;
    attachInterrupt(interruptNumber, eventHandler, CHANGE);
    return true;
  };
  
  // Utility function to give number of ticks since the last event.  Useful 
  //  for determining how much time has elapsed within the interrupt handler since
  //  the interrupt was triggered.
  inline unsigned long INTERRUPT_SAFE elapsedTicksSinceLastEvent() {
    return micros() - thisEventTicks;
  };
  
  // Function called from the interrupt handler to calculate the gap between interrupts,
  //  and to invoke the user program's handler.  The user's handler is passed the 
  //  number of ticks elapsed since the last valid interrupt.  It returns true/false to 
  //  indicate if this interrupt is deemed to be 'valid' or not.
  void INTERRUPT_SAFE processInterrupt(unsigned long thisEventTicks) {
    this->thisEventTicks = thisEventTicks;
    unsigned long eventSpacing = thisEventTicks - lastValidEventTicks;
    bool accepted = callUserHandler(eventSpacing);
    if (accepted) {
      lastValidEventTicks = thisEventTicks;
    }
  };

  // Utility function to return the number of timer ticks per microsecond.  
  //  With millis() as a counter, this is always 1.
  inline unsigned int ticksPerMicrosec() {
    return TICKSPERMICROSEC;
  };

  // Utility function to inform whether input capture is in use or not.  For this
  //  version, it always returns false.
  inline bool inputCaptureMode() { return false; };

private:
  EventHandler *callUserHandler = 0;
  unsigned long lastValidEventTicks = 0;
  unsigned long thisEventTicks = 0;
  int pin = -1;
} /* class EventTimerClass */;

// Declare singleton class instance
EventTimerClass EventTimer;

// Interrupt handler for digital input change event.
void INTERRUPT_SAFE eventHandler() {
  EventTimer.processInterrupt(micros());
}

#endif
