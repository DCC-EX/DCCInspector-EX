/*
 * OledDisplay class
 * 
 * Handles the Oled Display output from the DCC diagnostic program.
 * 
 * The output to be displayed is passed to the program as parameter
 * to the function update().  The string is sent to the screen (at least
 * as much as fits on the screen).  The buffer reference is held so that,
 * if the user presses the button connected to BUTTONPIN, the screen is updated
 * starting at a later line in the buffer.  In this way, the user can 
 * scroll through the text.  
 * If the current scroll line number is greater than the number of lines
 * of text when a refresh is due, then the text is displayed from the 
 * beginning once more.
 * 
 * OLED ISN'T SUPPORTED ON ARDUINO NANO AND UNO because of the lack of memory.
 */ 

#ifndef oleddisplay_h
#define oleddisplay_h

#include "Config.h"

// Only compile if USE_OLED is defined.
// If not, then the entire contents are ignored by the compiler.
#ifdef USE_OLED

#include "DCCStatistics.h"
#include "StringBuilder.h"

#include <Adafruit_SSD1306.h>

class OledDisplayClass {
public:
  // Function called to initialise the object instance.  
  //  Connects to the OLED display and puts it into a 
  //  suitable mode.
  bool begin(int sdaPin, int sclPin);
  void reset() {  
    sbOled.reset(); 
  }

  // Function called to force an update of the screen,
  //  starting with the line referenced by 'firstLine'.
  void refresh();

  // Check the button pin input to see if the button has
  //  been pressed.  If so, increment 'firstLine' and refresh,
  //  so that the displayed text scrolls upwards.
  // This function should be called regularly from the loop() function.
  void checkButton();

  // Display specified text on the screen.
  void append(const char *string);

  // Write a summary version of the statistics data onto the screen.
  void writeShortStatistics(Statistics &lastStats);

private:
  // OLED Display
  Adafruit_SSD1306 display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  // Line number (starting with 0) of first line to be displayed.
  int firstLine = 0;
  
  const char *selectedLine();

  char sbBuffer[1000];
  StringBuilder sbOled = StringBuilder(sbBuffer, sizeof(sbBuffer));

} /* class OledDisplayClass */;

extern OledDisplayClass OledDisplay;

#endif
#endif
