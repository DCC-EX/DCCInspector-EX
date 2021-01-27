/*
 * EventTimer.h
 * 
 * Include file that selects one of the specific EventTimer
 * class definitions, depending on architecture.
 * 
 */

#if defined(ARDUINO_UNO_NANO) || defined(ARDUINO_AVR_MEGA)
  #include "EventTimer_AtMega.h"
#elif defined(ESP32)
  #include "EventTimer_ESP32.h"
#else
  #include "EventTimer_default.h"
#endif
