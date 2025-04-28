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

// Module containing singleton instance of DCCStatistics class.

#include "DCCStatistics.h"

// Update statistics to reflect the received digital input transition.  Altbit
//  is zero if the transition is the end of the first half-bit and one if it is
//  the end of the second half of a DCC bit; Bitvalue is the value of the DCC
//  bit; and interruptInterval is the microsecond time between successive
//  interrupts (the length of the half-bit in DCC terms).
void INTERRUPT_SAFE DCCStatisticsClass::recordHalfBit(
    byte altbit, byte bitValue, unsigned int interruptInterval,
    unsigned int delta) {
  activeStats.count++;
  if (bitValue == 0) {
    activeStats.count0++;
    if (interruptInterval > activeStats.max0)
      activeStats.max0 = interruptInterval;
    if (interruptInterval < activeStats.min0)
      activeStats.min0 = interruptInterval;
    activeStats.total0 += interruptInterval;
    if (altbit && (delta > activeStats.max0BitDelta))
      activeStats.max0BitDelta = delta;
  } else {
    activeStats.count1++;
    if (interruptInterval > activeStats.max1)
      activeStats.max1 = interruptInterval;
    if (interruptInterval < activeStats.min1)
      activeStats.min1 = interruptInterval;
    activeStats.total1 += interruptInterval;
    if (altbit & (delta > activeStats.max1BitDelta))
      activeStats.max1BitDelta = delta;
  }
  if (interruptInterval < minBitLength)
    interruptInterval = minBitLength;
  else if (interruptInterval > maxBitLength)
    interruptInterval = maxBitLength;
  activeStats.countByLength[altbit][interruptInterval - minBitLength]++;
}

//=======================================================================
// WriteFullStatistics writes the statistics to Serial stream.
//
void DCCStatisticsClass::writeFullStatistics(Statistics &stats,
                                             bool showCpuStats,
                                             bool showBitLengths) {
  Serial.print(F("Bit Count/"));
  Serial.print(refreshTime);
  Serial.print(F(" sec="));
  // These counts are for half-bits, so divide by two.
  Serial.print(stats.count / 2);
  Serial.print(F(" (Zeros="));
  Serial.print(stats.count0 / 2);
  Serial.print(F(", Ones="));
  Serial.print(stats.count1 / 2);
  Serial.print(F("), Glitches="));
  Serial.println(stats.glitchCount);

  Serial.print(F("Valid Packets="));
  Serial.print(stats.packetCount);
  Serial.print(F(", Idles="));
  Serial.print(stats.idleCount);
  Serial.print(F(", NMRA out of spec="));
  Serial.print(stats.outOfSpecRejectionCount);
  Serial.print(F(", RCN 5ms errors="));
  Serial.print(stats.rcn5msFailCount);
  Serial.print(F(", Checksum Errors="));
  Serial.print(stats.checksumError);
  Serial.print(F(", Lost pkts="));
  Serial.print(stats.countLostPackets);
  Serial.print(F(", Long pkts="));
  Serial.println(stats.countLongPackets);

  Serial.print(F("0 half-bit length (us): "));
  if (stats.min0 <= stats.max0) {
    Serial.print((float)stats.total0 / stats.count0, 1);
    Serial.print(F(" ("));
    Serial.print(stats.min0);
    Serial.print(F("-"));
    Serial.print(stats.max0);
    Serial.print(F(")"));
    Serial.print(F(" delta < "));
    Serial.print(stats.max0BitDelta);
  } else
    Serial.print(F("<none>"));
  Serial.println();
  Serial.print(F("1 half-bit length (us): "));
  if (stats.min1 <= stats.max1) {
    Serial.print((float)stats.total1 / stats.count1, 1);
    Serial.print(F(" ("));
    Serial.print(stats.min1);
    Serial.print(F("-"));
    Serial.print(stats.max1);
    Serial.print(F(")"));
    Serial.print(F(" delta < "));
    Serial.print(stats.max1BitDelta);
  } else
    Serial.print(F("<none>"));
  Serial.println();

  if (showCpuStats) {
    Serial.print(F("IRC Duration (us): "));
    if (stats.minInterruptTime <= stats.maxInterruptTime) {
      Serial.print((float)stats.totalInterruptTime / stats.count, 1);
      Serial.print(F(" ("));
      Serial.print(stats.minInterruptTime);
      Serial.print(F("-"));
      Serial.print(stats.maxInterruptTime);
      Serial.print(F(")"));
    } else 
      Serial.print(F("<none>"));

    // Calculate and display cpu load
    unsigned long spareLoopCountPerSec = stats.spareLoopCount / refreshTime;
    Serial.print(F(",  CPU load: "));
    Serial.print(
        100.0f * (1.0f - (float)spareLoopCountPerSec / maxSpareLoopCountPerSec),
        1);
    Serial.print(F("%"));
    Serial.println();
  }

  if (showBitLengths) {
    Serial.println(F("------ Half-bit count by length (us) -------"));
    for (int i = minBitLength; i <= maxBitLength; i++) {
      unsigned long c0 = stats.countByLength[0][i - minBitLength];
      unsigned long c1 = stats.countByLength[1][i - minBitLength];
      if (c0 > 0 || c1 > 0) {
        if (i == minBitLength)
          Serial.print(F("<="));
        else if (i == maxBitLength)
          Serial.print(F(">="));
        Serial.print(i);
        Serial.print('\t');
        Serial.print(c0);
        Serial.print('\t');
        Serial.println(c1);
      }
    }
    Serial.println(F("--------------------------------------------"));
  }
}

// Return a copy of the current set of statistics accumulated.  We could inhibit
// interrupts while accessing the activeStats data, but the effect on the
// interrupt code may be more significant than the effect on the resulting
// counters.
Statistics DCCStatisticsClass::getAndClearStats() {
  Statistics stats;
  memcpy(&stats, (void *)&activeStats, sizeof(activeStats));
  memset((void *)&activeStats, 0, sizeof(activeStats));
  activeStats.minInterruptTime = activeStats.min0 = activeStats.min1 = 65535;

  // Sample spare loop count and adjust max accordingly, for CPU load calcs.
  if (maxSpareLoopCountPerSec < stats.spareLoopCount / refreshTime)
    maxSpareLoopCountPerSec = stats.spareLoopCount / refreshTime;
  stats.refreshTime = refreshTime;
  return stats;
}

// Declare singleton class instance
DCCStatisticsClass DCCStatistics;
