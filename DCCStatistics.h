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
 * Class definition to encapsulate the statistics recorded by the
 * program.  It has functions which are called to update the statistics
 * of each type of event (new interrupt, packet received, packet errors
 * etc.).
 * It also has functions to format and write the statistics to an output
 * stream inheriting from class Print.
 */

#ifndef dccstats_h
#define dccstats_h

#include <Arduino.h>
#include "Config.h"

#if defined(ESP32) || defined(ESP8266) || defined(ESP_PLATFORM)
#define INTERRUPT_SAFE IRAM_ATTR
#else
#define INTERRUPT_SAFE
#endif

// Range of bit lengths (us) for recording count by bit length.
const int minBitLength = 45; // microseconds (58 - 20% - 1)
const int maxBitLength = 141; // microseconds (116 + 20% + 1)

// Statistics structure.  Sizes of counts are chosen to be large enough not to overflow
typedef struct {
  unsigned int refreshTime;
  unsigned long count=0, count0=0, count1=0;
  unsigned int packetCount=0, checksumError=0, countLongPackets=0, countLostPackets=0;
  unsigned int outOfSpecRejectionCount = 0;
  unsigned int max1=0, min1=65535, max0=0, min0=65535;
  unsigned long total1=0, total0=0, totalInterruptTime=0;
  unsigned int maxInterruptTime=0, minInterruptTime=65535;
  unsigned int max1BitDelta=0, max0BitDelta=0;
  unsigned long glitchCount=0, spareLoopCount=0, innerSpareLoopCount=0;
  unsigned long rcn5msFailCount=0, idleCount=0;
  unsigned int countByLength[2][maxBitLength-minBitLength+1];
} Statistics;

class DCCStatisticsClass {
public: 

  // Update statistics to reflect the received digital input transition.  Altbit
  //  is zero if the transition is the end of the first half-bit and one if it is the
  //  end of the second half of a DCC bit; Bitvalue is the value
  //  of the DCC bit; and interruptInterval is the microsecond time between
  //  successive interrupts (the length of the half-bit in DCC terms).
  void INTERRUPT_SAFE recordHalfBit(byte altbit, byte bitValue, unsigned int interruptInterval, unsigned int delta);
  inline void INTERRUPT_SAFE recordLostPacket() {
    activeStats.countLostPackets++;
  }
  inline void INTERRUPT_SAFE recordLongPacket()  {
    activeStats.countLongPackets++;
  }
  inline void INTERRUPT_SAFE recordPacket() {
    activeStats.packetCount++;
  }
  inline void INTERRUPT_SAFE recordChecksumError() {
    activeStats.checksumError++;
  }
  inline void INTERRUPT_SAFE recordOutOfSpecRejection() {
    activeStats.outOfSpecRejectionCount++;
  }
  inline void INTERRUPT_SAFE recordrcn5msFailure() {
    activeStats.rcn5msFailCount++;
  }
  inline void INTERRUPT_SAFE recordIdlePacket() {
    activeStats.idleCount++;
  }
  inline void INTERRUPT_SAFE recordInterruptHandlerTime(unsigned int interruptDuration) {
    if (interruptDuration > activeStats.maxInterruptTime) activeStats.maxInterruptTime = interruptDuration;
    if (interruptDuration < activeStats.minInterruptTime) activeStats.minInterruptTime = interruptDuration;
    activeStats.totalInterruptTime += interruptDuration;
  }
  inline void INTERRUPT_SAFE recordGlitch() {
    activeStats.glitchCount++;
  }

  inline void updateLoopCount() {
    activeStats.spareLoopCount++; 
  }
  inline bool faultPresent() {
    if (activeStats.glitchCount > 0 || activeStats.checksumError > 0 || 
        activeStats.countLongPackets > 0 || activeStats.countLostPackets > 0
        || activeStats.outOfSpecRejectionCount > 0
        || activeStats.rcn5msFailCount > 0)
        return true;
    else
        return false;
  }
  void writeFullStatistics(Statistics &stats, bool showCpuStats, bool showBitLengths);
  void writeShortStatistics(Print &output);
  
  // Return a copy of the current set of statistics accumulated.
  Statistics getAndClearStats();

  inline unsigned int getRefreshTime() { return refreshTime; }
  inline void setRefreshTime(unsigned int value) { refreshTime = value; }

private:
  unsigned long maxSpareLoopCountPerSec = 0; // baseline for CPU load calculation
  unsigned int refreshTime = 4;
  volatile Statistics activeStats;  // Statistics currently being updated
};

extern DCCStatisticsClass DCCStatistics;

#endif
