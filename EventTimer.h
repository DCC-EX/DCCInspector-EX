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
 * EventTimer.h
 * 
 * Include file that selects one of the specific EventTimer
 * class definitions, depending on architecture.
 * 
 */

#if defined(ARDUINO_UNO_NANO) || defined(ARDUINO_AVR_MEGA2560)
  #include "EventTimer_AtMega.h"
#elif defined(ESP32)
  #include "EventTimer_ESP32.h"
#else
  #include "EventTimer_default.h"
#endif
