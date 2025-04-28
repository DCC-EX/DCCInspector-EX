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
 * Class to encapsulate the handling of the OLED display in the 
 * DCC Diagnostic program.  It also handles the button which 
 * is used for scrolling the display.
 */

#include "OledDisplay.h"

#include "Config.h"

#ifdef USE_OLED

// Function called to initialise the object instance.  
//  Connects to the OLED display and puts it into a 
//  suitable mode.
bool OledDisplayClass::begin(int sdaPin, int sclPin) {
    Serial.print("OLED SDA/SCL=");
    Serial.print(sdaPin);
    Serial.print("/");
    Serial.println(sclPin);
    #if defined(ESP32) || defined(ESP8266)
    Wire.begin(sdaPin, sclPin);
    #endif
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2CADDRESS)) {
        Serial.println("Can't connect to OLED display!");
        return false;
    }
    display.clearDisplay();
    display.display();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
    //display.setRotation(2);

    // Initialise the button pin.
    #ifdef BUTTONPIN
    pinMode(BUTTONPIN, INPUT_PULLUP);
    #endif
    
    Serial.println("OLED initialised");
    return true;
}

// Function called to force an update of the screen,
//  starting with the line referenced by 'firstLine'.
void OledDisplayClass::refresh() {
    // locate nominated line
    display.clearDisplay();
    display.setCursor(0,0);
    display.print(selectedLine());
    display.display();
}

// Check the button pin input to see if the button has
//  been pressed.  If so, increment 'firstLine' and refresh,
//  so that the displayed text scrolls upwards.
// This function should be called regularly from the loop() function.
void OledDisplayClass::checkButton() {
    #ifdef BUTTONPIN
    static int lastButtonState = 0;
    int buttonState = digitalRead(BUTTONPIN);
    if (buttonState == 0 && lastButtonState == 1) {
        // Button pressed.  Move a few lines down.
        firstLine += 4;
        refresh();
    }
    lastButtonState = buttonState;
    #endif
}

// Display specified text on the screen.
void OledDisplayClass::append(const char *string) {
    sbOled.println(string);
    sbOled.end();
    refresh();
}

//=======================================================================
// WriteShortStatistics sends a short summary to a print stream for writing 
// to the OLED display.

void OledDisplayClass::writeShortStatistics(Statistics &lastStats) {

    sbOled.reset();
    if (lastStats.count > 0) {
        sbOled.print(F("Bits/"));
        sbOled.print(lastStats.refreshTime);
        sbOled.print(F(" sec: "));
        sbOled.println(lastStats.count/2);

        sbOled.println(F("Lengths (us)"));

        sbOled.print(F(" 0:"));
        if (lastStats.count0 > 0) {
        sbOled.print((double)lastStats.total0/lastStats.count0, 1);
        sbOled.print(F(" ("));
        sbOled.print(lastStats.min0);
        sbOled.print('-');
        sbOled.print(lastStats.max0);
        sbOled.println(')');
        } else 
        sbOled.println(F("N/A"));

        sbOled.print(F(" 1:"));
        if (lastStats.count1 > 0) {
        sbOled.print((double)lastStats.total1/lastStats.count1, 1);
        sbOled.print(F(" ("));
        sbOled.print(lastStats.min1);
        sbOled.print('-');
        sbOled.print(lastStats.max1);
        sbOled.println(')');
        } else 
        sbOled.println(F("N/A"));
        
        sbOled.print(F("Deltas: 0:<"));
        sbOled.print(lastStats.max0BitDelta);
        sbOled.print(F(" 1:<"));
        sbOled.println(lastStats.max1BitDelta);

        sbOled.print(F("Glitches: "));
        sbOled.println(lastStats.glitchCount);

        sbOled.print(F("Frames: "));
        sbOled.println(lastStats.packetCount);
        
        sbOled.print(F("CksumErr: "));
        sbOled.println(lastStats.checksumError);
        
        sbOled.print(F("NMRA Reject: "));
        sbOled.println(lastStats.outOfSpecRejectionCount);
        
        sbOled.print(F("RCN 5ms Fails: "));
        sbOled.println(lastStats.rcn5msFailCount);
        sbOled.println(F("--"));
        
    } else {
        sbOled.print(F("Bits/"));
        sbOled.print(lastStats.refreshTime);
        sbOled.println(F(" sec: 0\n"
        " 0: 0 1: 0\n"
        "Lengths (us)\n"
        " 0: N/A\n"
        " 1: N/A\n"
        "Deltas: N/A\n"
        "Frames: 0\n"
        "CksumErr: 0\n"
        "NMRA Reject: 0\n"
        "--"));
    }
    sbOled.end();
}

// Locate pointer to first character of line 'firstLine' in buffer 'sbBuffer'.
// If line doesn't exist, then go back to the beginning of the buffer.
const char *OledDisplayClass::selectedLine() {
    if (firstLine > 0) {
        int thisLineNo = 0;
        for (const char *cptr=sbBuffer; *cptr != 0; cptr++) {
            if (*cptr == '\n') {
                if (++thisLineNo == firstLine) {
                    cptr++;  // Move to start of next line following newline
                if (*cptr == '\0')
                    break;  // Gone off end...
                else
                    return cptr; // line valid.
                }
            }
        }
    }
    // If we get here then there aren't enough lines, so go back to start.
    firstLine = 0;
    return sbBuffer;
}


// Singleton instance of class
OledDisplayClass OledDisplay;

#endif