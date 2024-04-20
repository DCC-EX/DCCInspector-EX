/* Copyright (c) 2024 Heiko Rosemann
 * Based on EventTimer_AtMega.h (c) 2021 Neil McKechnie
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
 * EventTimer_AtMega0series.h
 * 
 *  Timer functions for measuring elapsed time between events.  On the mega0-series,
 *  the functions use the Input Capture capability of the microcontroller to capture the
 *  timer value very precisely using hardware.
 *
 *  The pin used as input pin can be configured in Config.h
 * 
 *  Since the Timer counter is limited to 16 bits, it will overflow if the interval exceeds 
 *  about 32milliseconds.  To avoid this, we check micros() to determine if pulse 
 *  length is too long and, in that case, calculate the ticks from micros().
 *  
 */

#ifndef eventtimer_default_h
#define eventtimer_default_h

#include <Arduino.h>
#include "Config.h"
#include "Event.h"

#define TICKSPERMICROSEC 8   // 8 (125ns) or 16 (62.5ns)

#if not defined(ARDUINO_NANO_EVERY)
  #error "Architecture not supported by EventTimer library."
#endif

/* 
 *  User event handler is sent the time since the last valid event.
 *  If the user event handler decides that this event isn't valid 
 *  (e.g. time since last one is too short) then it should return 
 *  false to reject the event.  If the event is valid, then it 
 *  should return true to accept it.
 */
typedef bool EventHandler(unsigned long eventInterval);

class EventTimerClass {
public:

  // Initialise the object instance, validating that the input pin is
  //  correct and noting the reference to the user handler for future use.
  bool begin(int pin, EventHandler userHandler) {
#if defined(GPIO_PREFER_SPEED)
    if (pin != INPUTPIN) {
      Serial.println(F("ERROR: Cannot use fast GPIO with pin != INPUTPIN"));
      Serial.print(F("pin="));
      Serial.println(pin);
      Serial.print(F("INPUTPIN="));
      Serial.println(INPUTPIN);
      return false;
    }
#endif
    Event & myEvent = Event::assign_generator_pin(pin);
    if (myEvent.get_channel_number() == 255) {
      Serial.print(F("ERROR: pin=")); 
      Serial.print(pin); 
      return false;
    }

    myEvent.set_user(event::user::tcb2_capt);
    
    this->pin = pin;
    this->callUserHandler = userHandler;

    // Set up input capture.
    // Configure Timer to increment TCNTn on an 8MHz clock,
    // (every 0.5us), or 16MHz (every 62.5ns), no interrupt.
    // Use Input Capture Pin ICP to capture time of input change
    // and interrupt the CPU.
    #if TICKSPERMICROSEC==8
    TCB2.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm; // Prescaler CLK/2
    #elif TICKSPERMICROSEC==16
    TCB2.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; // Prescaler CLK/1
    #else
    #error "TICKSPERMICROSEC" not 8 or 16.
    #endif
    TCB2.CTRLB = TCB_CNTMODE_CAPT_gc;            // Setup Timer B2 for capture mode
    TCB2.EVCTRL = TCB_FILTER_bm | TCB_CAPTEI_bm; // Enable input capture filter and event
    TCB2.INTCTRL = TCB_CAPT_bm;                  // Enable input capture interrupt
    TCB2.INTFLAGS = TCB_CAPT_bm;                 // Clear interrupt flag

    myEvent.start();
    
    return true;
  };
  
  // Utility function to give number of ticks since the last event.  Useful 
  //  for determining how much time has elapsed within the interrupt handler since
  //  the interrupt was triggered.
  inline unsigned long elapsedTicksSinceLastEvent() {
    return (unsigned int)(TCB2.CNT - thisEventTicks);
  };
  
  // Function called from the interrupt handler to calculate the gap between interrupts,
  //  and to invoke the user program's handler.  The user's handler is passed the 
  //  number of ticks elapsed since the last valid interrupt.  It returns true/false to 
  //  indicate if this interrupt is deemed to be 'valid' or not.
  void processInterrupt() {
    // Time-critical bits.
    unsigned long thisEventMicros = micros();
    thisEventTicks = TCB2.CCMP;
    byte diginState = digitalRead(INPUTPIN);

    // Set up input capture for next edge.
    if (diginState) 
      TCB2.EVCTRL |= TCB_EDGE_bm; // Capture next falling edge on input
    else
      TCB2.EVCTRL &= ~TCB_EDGE_bm; // Capture next rising edge on input

    // Initially estimate time ticks using micros() values.
    unsigned long eventSpacing = (thisEventMicros - lastValidEventMicros) * TICKSPERMICROSEC;
    if (eventSpacing < 60000L) {
      // Estimated time is well within range of timer count, so calculate ticks from ICRn
      eventSpacing = thisEventTicks - lastValidEventTicks;
    }
    // Call user handler.
    bool accepted = callUserHandler(eventSpacing);
    if (accepted) {
      lastValidEventTicks = thisEventTicks;
      lastValidEventMicros = thisEventMicros;
    }
  };

  // Utility function to return the number of timer ticks per microsecond.  
  //   On the AtMega 0 series, this is 8 or 16, depending on the timer pre-scaler setting.
  inline unsigned int ticksPerMicrosec() {
    return TICKSPERMICROSEC;
  };

  // Utility function to inform whether input capture is in use or not.  For this
  //  version, it always returns true.
  inline bool inputCaptureMode() { return true; };

private:
  EventHandler *callUserHandler = 0;
  unsigned int lastValidEventTicks = 0;
  unsigned long lastValidEventMicros = 0;
  unsigned int thisEventTicks = 0;
  int pin = -1;

} /* class EventTimerClass */;

// Declare singleton class instance.
EventTimerClass EventTimer;

// Interrupt handler for input capture event
ISR(TCB2_INT_vect) {
  EventTimer.processInterrupt();
}

#endif
