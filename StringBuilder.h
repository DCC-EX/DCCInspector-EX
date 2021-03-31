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
 * StringBuilder class
 * 
 * Inherits from Print class so allows all Print class methods to write to 
 * a char[] buffer.
 * 
 */
#ifndef stringbuilder_h
#define stringbuilder_h

#include <Arduino.h>
#include <Print.h>

class StringBuilder : public Print {
public:
    // Constructor
    StringBuilder(char *buf, size_t len) : Print() {
        buffer = buf; 
        buffMax = len-1;  // Allow space for null terminating character.
        reset();
    }

    // Reset (empty) buffer.
    void reset() {
        buffIndex = 0;
        overflow = false;
        end();
    }

    // Pad with spaces to the nominated position.
    void setPos(unsigned int pos) {
        int n = pos - buffIndex;    // Number of spaces to write
        for (int i=0; i<n; i++)
            write(' ');
    }

    // Write a character.
    size_t write(uint8_t c) {
        if (buffIndex+3 < buffMax) {
            buffer[buffIndex++] = c;
            return 1;
        } else {
            overflow = true;
            return 0;
        }
    }

    // Write an array of characters.
    size_t write(const uint8_t *buffer, size_t size) {
        for (unsigned int i=0; i<size; i++) {
            if (!write(*buffer++))
                return i;
        }
        return size;
    }

    // Append a null character, to terminate the string 
    // in the buffer.  Position isn't updated, so further
    // characters can be appended to the string if required
    // (and if there's space).
    void end() {
        // There is still room for three characters plus a terminator.
        // If the buffer has filled, then add three dots.
        if (overflow && buffIndex+3 < buffMax) {
            buffer[buffIndex++] = '.';
            buffer[buffIndex++] = '.';
            buffer[buffIndex++] = '.';
        }
        buffer[buffIndex] = '\0'; // terminate string
    }

    // Get a pointer to the string within the buffer.
    char *getString() {
        end();
        return buffer;
    }

private:
    char *buffer = 0;
    unsigned int buffMax = 0;
    unsigned int buffIndex = 0;
    bool overflow = false;
};

#endif