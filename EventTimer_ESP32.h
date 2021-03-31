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
 * EventTimer_ESP32.h
 * 
 *  Timer functions for measuring elapsed time between events.  On the ESP32 platform,
 *  the functions use the os-provided input capture support functions.
 */

#ifndef eventtimer_default_h
#define eventtimer_default_h

#include <Arduino.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_struct.h"
#include "soc/mcpwm_reg.h"

#define TICKSPERMICROSEC 80

/* 
 *  User event handler is sent the time since the last valid event.
 *  If the user event handler decides that this event isn't valid 
 *  (e.g. time since last one is too short) then it should return 
 *  false to reject the event.  If the event is valid, then it 
 *  should return true to accept it.
 *  For the ESP32 (and ESP8266) this must be declared IRAM_ATTR.
 */
typedef bool EventHandler(unsigned long eventInterval);

static void IRAM_ATTR isr_handler(void *);

class EventTimerClass {
public:

  // Initialise the object instance, validating that the input pin is
  //  correct and noting the reference to the user handler for future use.
  bool begin(int pin, EventHandler userHandler) {
    this->pin = pin;
    this->callUserHandler = userHandler;

    // In the version of the ESP32 SDK included with the Arduino platform, there doesn't seem to be a way
    // of capturing on both edges; only on positive edge or negative edge.  Therefore, we're 
    // setting up two separate captures, one on each edge. 
    // When the input pin changes state, the current value of the timer is captured and then the interrupt
    // is scheduled.  The interrupt response code is responsible for retrieving the captured value.
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, INPUTPIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, INPUTPIN);
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  
            //capture signal on positive edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_NEG_EDGE, 0);  
            //capture signal on negative edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    MCPWM0.int_ena.cap0_int_ena = 1;                            // Enable interrupt on CAP0 signal
    MCPWM0.int_ena.cap1_int_ena = 1;                            // Enable interrupt on CAP1 signal
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL); // Set ISR Handler
    return true;
  };

  // Utility function to give number of ticks since the last event.  Useful 
  //  for determining how much time has elapsed within the interrupt handler since
  //  the interrupt was triggered.
  //  I couldn't find a way of retrieving the current ICP timer value, only the captured one.
  //  Until I can find one, here's a work-around that uses the micros() function.  It's not 
  //  used for timing events, so isn't particularly critical.
  inline unsigned long IRAM_ATTR elapsedTicksSinceLastEvent() {
    return (micros() - thisEventMicros) * TICKSPERMICROSEC;
  };

  // Function called from the interrupt handler to calculate the gap between interrupts,
  //  and to invoke the user program's handler.  The user's handler is passed the 
  //  number of ticks elapsed since the last valid interrupt.  It returns true/false to 
  //  indicate if this interrupt is deemed to be 'valid' or not.
   void IRAM_ATTR processInterrupt() {
    // Get current micros() value to support elapsedTicksSinceLastEvent().
    thisEventMicros = micros();

    // Check which interrupt is pending.
    mcpwm_capture_signal_t capture = MCPWM_SELECT_CAP0;
    if (MCPWM0.int_st.cap0_int_st) {
      capture = MCPWM_SELECT_CAP0;
      MCPWM0.int_clr.cap0_int_clr = 1; // Clear capture 0 interrupt
    } else if (MCPWM0.int_st.cap1_int_st) {
      capture = MCPWM_SELECT_CAP1;
      MCPWM0.int_clr.cap1_int_clr = 1; // Clear capture 1 interrupt
    } else
      return;         // No interrupt pending.
      
    //unsigned long edge = mcpwm_capture_signal_get_edge(MCPWM_UNIT_0, capture); // 1=positive edge, 2=negative edge

    unsigned long thisEventTicks = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, capture); //get capture signal counter value
    unsigned long eventInterval = thisEventTicks - lastValidEventTicks;
    bool accepted = callUserHandler(eventInterval);
    if (accepted) {
      lastValidEventTicks = thisEventTicks;
    }
  };

  // Utility function to return the number of timer ticks per microsecond.  
  // On the ESP32, this is normally 80 but can be adjusted through a prescaler.
  inline unsigned int ticksPerMicrosec() {
    return TICKSPERMICROSEC;
  };

  // Utility function to inform whether input capture is in use or not.  For this
  //  version, it always returns true.
  inline bool inputCaptureMode() { return true; };

private:
  EventHandler *callUserHandler = 0;
  unsigned long lastValidEventTicks = 0;
  unsigned long thisEventMicros = 0;
  int pin = -1;
} /* class EventTimerClass */;

EventTimerClass EventTimer;

static void IRAM_ATTR isr_handler(void *) {
  EventTimer.processInterrupt();
}

#endif
