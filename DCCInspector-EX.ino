/////////////////////////////////////////////////////
//
// Use ICR for pulse measurements, and display pulse length histogram: NMcK Dec 2020
// Add CPU load figures for the controller: NMcK December 2020
// Count, and optionally filter, edge noise from input: NMcK December 2020
// Display half-bit lengths rather than bit lengths: NMcK December 2020
// Improved comments; moved some strings to flash: NMcK November 2020
// Moved frame construction into interrupt code: NMcK July 2020
// Removed use of AVR timers: NMcK June 2020
// DCC packet analyze: Ruud Boer, October 2015
//
// The DCC signal is detected on Arduino digital pin 8 (ICP1 on Uno/Nano),
// or pin 49 (ICP4 on Mega), or pin 2.  The use of ICPn enables accurate
// timing to within 1us; pin 2 just uses the millis() function which is no better 
// than 10.5us or so (6.5us interrupt jitter and 4us timer resolution).  
// Use an opto-isolator between the track signal (~30V p-p) and the 
// Arduino digital input (0 to 5V) to prevent burn-out.
//
// Written originally for Uno but tested on Mega and Nano,
// should work on others if simple options chosen.  Compiles on ESP8266 but 
// to date hasn't been tested.
//
// It needs the selected digital input to have interrupt support, either via 
// a level change interrupt (e.g. pin 2) or input capture interrupt (e.g. pin 8
// on Arduino Uno).
//
// The bit decoding is done by measuring the time between successive 
// interrupts using the 'micros()' function (or, optionally, using a faster 
// 16-bit timer for higher resolution) so should be pretty portable.
//
// The counter SpareLoopCount is used to see how heavily loaded the processor is.
// With DCC interrupts off, it measures the count.  When the interrupts are attached
// it sees how much the count drops.  The percent load due to the interrupts is 
// the percentage difference between the two figures.
// For example (my measurements on an Uno):
//    DCC off, SpareLoopCount=1043042 (over 4 secs)
//    DCC on, SpareLoopCount=768830 (over 4 secs)
//    CPU Load = (1043042-766830)/1043042*100 = 26%
//
// Loco address 3's speed command is mapped onto PWM output pin 3, allowing an 
// LED to be used to confirm that a controller is able to send recognisable 
// DCC commands.
//
// Set the Serial Monitor Baud Rate to 115200 !!
//
// Keyboard commands that can be sent via Serial Monitor:
// 1 = 1s refresh time
// 2 = 2s 
// 3 = 4s (default)
// 4 = 8s
// 5 = 16s
// 6 = 4 DCC packet buffer
// 7 = 8
// 8 = 16
// 9 = 32 (default)
// 0 = 64
// a = show accessory packets toggle
// l = show locomotive packets toggle
// d = show diagnostics toggle
// h = show heartbeat toggle
// b = show half-bit counts by pulse length
// f = filter input toggle
// ? = help (show this information)
//
////////////////////////////////////////////////////////

// Uncomment the following line to use the 'Fast digital i/o library' DIO2
// in place of the normal digitalRead() and digitalWrite().
// Reduces CPU load by about 17%.
#define USE_DIO2

// Uncomment the "#define USETIMER" to perform the timing using the ATmega328's 
// 16-bit timer instead of micros(). This enables a 1us resolution rather than 4us.
// It also reduces load by about 7%, and because we can capture the 
// Timer counter register (TCNTn) at the time of the digital change, it is immune to 
// timing errors caused by other interrupts.
// Works on Arduino Uno (Timer1/pin D8)
//          Arduino Nano (Timer1/pin D8)
//          Arduino Mega (Timer4/pin D49)
// If we don't use this, then the selected input pin must support change interrupt 
// (defaults to pin D2 on Uno, Nano and Mega).
#define USETIMER 

// LED pin definitions.
#define LEDPIN_ACTIVE 13   // Shows interrupts being received, ie DCC active
#define LEDPIN_LOCOSPEED 3  // Driven from loco speed packet for loco 3
//#define LEDPIN_DECODING 5  // lights when a packet with valid checksum is received
#define LEDPIN_FAULT 6   // Lights when a checksum error or glitch is encountered.

#define SERIAL_SPEED 115200

// Comment out following line to eliminate diagnostic code
// and make the interrupt code faster.  However, the diagnostic
// code tells you if the DCC timing is healthy and if the sniffer
// is correctly operating, so it is recommended that it is enabled.
#define SHOWSTATS 3

////////////////////////////////////////////////////////

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
  #if defined(USETIMER)
    #define INPUTPIN 8
    #define TCNTn TCNT1 // TimerN Counter Register
    #define ICRn ICR1   // TimerN Input Change Register
    #define TCCRnB TCCR1B // TimerN configuration register
    #define TIMERn_CAPT_vect TIMER1_CAPT_vect  // ISR vector
  #else
    #define INPUTPIN 2
  #endif
#elif defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_ADK)
  #if defined(USETIMER)
    #define INPUTPIN 49
    #define TCNTn TCNT4 // TimerN Counter Register
    #define ICRn ICR4   // TimerN Input Change Register
    #define TCCRnB TCCR4B // TimerN configuration register
    #define TIMERn_CAPT_vect TIMER4_CAPT_vect  // ISR vector
  #else
    #define INPUTPIN 2
  #endif
#else
  // Other architectures
  #define INPUTPIN 2
  #undef USE_DIO2  // Not supported
  #undef USETIMER  // Not supported
#endif

#if defined(USE_DIO2)
#define GPIO_PREFER_SPEED
#include <DIO2.h>
#define digitalRead(a) digitalRead2f(PIN(a))
#define digitalWrite(a,b) digitalWrite2f(PIN(a),b)
#define pinMode(a,b) pinMode2f(PIN(a),b)
#define PIN(n) DP##n
#else
#define PIN(n) n
#endif

const int nPackets=8; // Number of packet buffers
const int pktLength=8; // Max length+1 in bytes of DCC packet
const int minBitLength = 40; // microseconds
const int maxBitLength = 140; // microseconds

// Statistics structure.  Sizes of counts are chosen to be large enough not to overflow.
struct stats {
  unsigned long count=0, count0=0, count1=0;
  unsigned int packetCount=0, checksumError=0, countLongPackets=0, countLostPackets=0;
  unsigned int max1=0, min1=65535, max0=0, min0=65535;
  unsigned long total1=0, total0=0, totalInterruptTime=0;
  unsigned int maxInterruptTime=0, minInterruptTime=65535;
  unsigned long glitchCount=0, spareLoopCount=0;
  unsigned int countByLength[2][maxBitLength-minBitLength+1];
};

// Variables shared by interrupt routine and main loop
volatile byte dccPacket[nPackets][pktLength]; // buffer to hold packets 
volatile byte packetsPending=0; // Count of unprocessed packets
volatile byte activePacket=0;  // indicate which buffer is currently being filled
volatile bool filterInput = true; // conditions input to remove transient changes
#ifdef SHOWSTATS
volatile struct stats activeStats;
#endif
volatile bool enableStats = true;


// Variables used by main loop
byte refreshTime = 4; // Time between DCC packets buffer refreshes in secs
byte packetHashListSize = 32; // DCC packets checksum buffer size
bool showLoc=true;
bool showAcc=true;
bool showHeartBeat=true;
bool showDiagnostics=true;
bool showBitLengths=false;
byte inputPacket=0; // Index of next packet to be analysed in dccPacket array
byte pktByteCount=0;
byte instrByte1;
byte decoderType; //0=Loc, 1=Acc
byte isDifferentPacket=0;
int bufferCounter=0;
struct stats lastStats; // Snapshot of activeStats.
unsigned long maxSpareLoopCountPerSec = 0; // baseline for CPU load calculation
unsigned int decoderAddress;
unsigned int packetHashList[64];
bool calibrated = false;
unsigned long lastRefresh = 0;

//=======================================================================
// Perform the setup functions for the application.

void setup() {
  Serial.begin(SERIAL_SPEED);
  Serial.println(F("---"));
  Serial.print(F("DCC Packet Analyze initialising...  "));

  #ifdef USETIMER
  // Configure Timer1 to increment TCNT1 on a 2MHz clock,
  // i.e. every 0.5us (no interrupt).
  // Use Input Capture Pin ICP to capture time of input change.
  TCCR1A = 0;
  TCCR1B = (1 << CS11); // Prescaler CLK/8
  TCCR1B |= (1 << ICNC1); // Input capture noise canceller
  #endif

  #ifdef LEDPIN_ACTIVE
  pinMode(LEDPIN_ACTIVE, OUTPUT);
  #endif
  #ifdef LEDPIN_DECODING
  pinMode(LEDPIN_DECODING, OUTPUT);
  #endif
    #ifdef LEDPIN_DECODING
  pinMode(LEDPIN_DECODING, OUTPUT);
  #endif
  #ifdef LEDPIN_FAULT
  pinMode(LEDPIN_FAULT, OUTPUT);
  #endif
 
  // Enable pullup in case there's no external pullup resistor.
  // External resistor is preferred as it can be lower, so 
  // improve the switching speed of the Optocoupler.
  pinMode(INPUTPIN, INPUT_PULLUP);
  
  // Delay first output of statistics 
  lastRefresh = millis();
}

//=======================================================================
// Main program loop.  
  
void loop() {
  bool somethingDone = false;
  
  if (millis() >= lastRefresh + (unsigned long)refreshTime * 1000) {
    
    if (calibrated) {
      
      // Copy. reset, and output diagnostics.
      if (showHeartBeat) Serial.println('-');
      ClearDccData();
      OutputStatistics();
      
    } else { 
      
      // Calibration was in progress but is now finished.
      Serial.println(F("done."));
      Serial.print(F("Updates every "));
      Serial.print(refreshTime);
      Serial.println(F(" seconds"));
      Serial.println(F("---"));
      maxSpareLoopCountPerSec = activeStats.spareLoopCount / refreshTime;
      calibrated = true;
      // Start recording data from DCC.
      ClearDccData();
      beginBitDetection();
      
    } 
    lastRefresh = millis();
    somethingDone = true;
  }

  // Check for DCC packets - if found, analyse and display them
  somethingDone |= processDCC();

  // Check for commands received over the USB serial connection.
  somethingDone |= processCommands();
    
  //Increment CPU loop counter.  This is done if nothing else was.
  // If the counter never gets incremented, it means that the 
  // CPU is fully loaded doing other things and has no spare time.
  if (!somethingDone) activeStats.spareLoopCount++; 

  UpdateLED();
}

//=======================================================================
// ISR invoked on change of state of INPUTPIN.  
//  It measures the time between successive changes (half-cycle of DCC
//  signal).  Depending on the value, it decodes 0 or a 1 for alternate 
//  half-cycles.  A 0 bit is nominally 100us per half-cycle (NMRA says 90-10000us)
//  and a 1 bit is nominally 58us (52-64us).  We treat a half-bit duration < 80us 
//  as a '1' bit, and a duration >= 80us as a '0' bit. 
//  Prologue and framing bits are detected and stripped, and data bytes are
//  then stored in the packet queue for processing by the main loop.
//
#ifdef USETIMER
ISR(TIMERn_CAPT_vect) {
#else
void scan() {
#endif
  static byte preambleOneCount = 0;
  static boolean preambleFound = false;
  static int newByte = 0;   // Accumulator for input bits until complete byte found.
  static int inputBitCount = 0;   // Number of bits read in current newByte.
  static int inputByteNumber = 0;  // Number of bytes read into active dccPacket buffer so far
  static boolean firstInterrupt = true;
  static byte previousBitValue = 0, previousState = 0;
  static unsigned long previousInterruptTime = 0;
  static byte altbit = 0;   // 0 for first half-bit and 1 for second.
  byte bitValue, state;
  unsigned long currentInterruptTime;
  
  // The most critical parts are done first - read time of change, and state of digital input.
  currentInterruptTime = micros2_cap();    // POINTA
  state = digitalRead(INPUTPIN);

  // Measure time since last valid interrupt.  Only consider low 15 bits (up to 32.767 ms)
  unsigned int interruptInterval = (currentInterruptTime - previousInterruptTime) & 0x7fff;
  
  // Precondition input?
  if (filterInput) {
    // Check that the digital input has actually changed since last interrupt, and that
    // the gap between interrupts is realistic.
    if (!firstInterrupt && (state == previousState || interruptInterval <= 3)) {
      // No change in digital, or it was fleeting.  Ignore.
      #if SHOWSTATS >= 1
      activeStats.glitchCount++;
      #endif
      return;
    }
  }
 
  // If we get here, the interrupt looks valid, i.e. the digital input really did
  // change its state more than 3us after its last change.
  // Record timer and input state ready for next interrupt
  previousState = state;
  previousInterruptTime = currentInterruptTime;
  #ifdef USETIMER
  if (state) 
    TCCRnB &= ~(1 << ICES1); // Capture next falling edge on input
  else
    TCCRnB |= (1 << ICES1); // Capture next rising edge on input
  #endif

  // If first interrupt, don't do anything else.
  if (firstInterrupt) {
    firstInterrupt = false;
    return;
  }

  #ifdef LEDPIN_ACTIVE
  digitalWrite(LEDPIN_ACTIVE, 1);
  #endif

  // Check length of half-bit
  if (interruptInterval < 80) 
    bitValue = 1;
  else
    bitValue = 0;

  // Check if we're on the first or second half of the bit.
  if (bitValue != previousBitValue) {
    // First half of new bit received
    altbit = false;
  } else {
    // Toggle for alternate half-bits
    altbit = !altbit;
  }
  previousBitValue = bitValue;
  
  #if SHOWSTATS >= 1
  // Update stats.
  activeStats.count++;
  if (bitValue == 0) {
    activeStats.count0++;
    if (enableStats) {
      if (interruptInterval > activeStats.max0) activeStats.max0 = interruptInterval;
      if (interruptInterval < activeStats.min0) activeStats.min0 = interruptInterval;
    }
    activeStats.total0 += interruptInterval;
  } else {
    activeStats.count1++; 
    if (enableStats) {
      if (interruptInterval > activeStats.max1) activeStats.max1 = interruptInterval;
      if (interruptInterval < activeStats.min1) activeStats.min1 = interruptInterval;
    }
    activeStats.total1 += interruptInterval;
  }
  if (interruptInterval >= minBitLength && interruptInterval <= maxBitLength) 
    activeStats.countByLength[altbit][interruptInterval - minBitLength]++;
  #endif

  // If this is the second half-bit then we've got a whole bit!!
  if (altbit) {

    // Now we've got a bit, process it.  The message comprises the following:
    //   Preamble: 10 or more '1' bits followed by a '0' start bit.
    //   Groups of 9 bits each containing data byte of 8 bits, followed by a 
    //   '0' bit (if message not yet finished), or a '1' bit (if the byte is 
    //   the last byte of the message, i.e. the checksum).
    //
    if (!preambleFound) {
      if (bitValue==1) {
        // Reading preamble perhaps...
        preambleOneCount++;
      } else if (preambleOneCount < 10) { // and bitValue==0)
        // Preamble not yet found, but zero bit encountered.  Restart preable count.
        preambleOneCount = 0;
      } else { // preambleOneCount >= 10 and bitValue==0
        // Start bit found at end of preamble, so prepare to process data.
        preambleFound = true;
        newByte = 0;
        inputBitCount = 0;
        inputByteNumber = 0;
      }
    } else {   // Preamble previously found, so this is a message bit
      if (packetsPending==nPackets) {
        // Previous DCC packets haven't been processed by the main loop,
        // so there is no buffer for the incoming message. 
        // Discard incoming message and scan for another preamble.
        preambleFound = false;
        preambleOneCount = 0;
        #if SHOWSTATS >= 3
        // Record this event in a counter.
        activeStats.countLostPackets++;
        #endif
      } else {
        // Preamble read, packet buffer available, so message bit can be stored!
        if (inputBitCount == 8) { // Byte previously completed, so this bit is the interbyte marker
          if (bitValue==0) {  // Interbyte marker is zero, so prepare for next byte of data
            inputBitCount = 0;
          } else { // one-bit found, marks end of packet
            // End of packet found
            dccPacket[activePacket][0] = inputByteNumber; // save number of bytes
            packetsPending++; // flag that packet is ready for processing
            if (++activePacket >= nPackets) activePacket = 0;  // move to next packet buffer
            #if SHOWSTATS >= 3
            activeStats.packetCount++;
            #endif
            preambleFound = false; // scan for another preamble
            preambleOneCount = 1;  // allow the current bit to be counted in the preamble.
          }
        } else { // Reading packet data at this point.
          // Append received bit to the current new byte.
          newByte = (newByte << 1) | bitValue;
          if (++inputBitCount == 8) { // Completed byte, save byte (if room)
            if (inputByteNumber < pktLength-1)
              dccPacket[activePacket][++inputByteNumber] = newByte;
            else {  // packet has filled buffer so no more bits can be stored!
              packetsPending++; // flag that packet is ready for processing
              if (++activePacket >= nPackets) activePacket = 0;  // move to next packet buffer
              preambleFound = false;  // scan for another preamble
              preambleOneCount = 0; 
              #if SHOWSTATS >= 3
              // Record this event in a counter.
              activeStats.countLongPackets++;
              #endif
            }             
            newByte = 0;
          }
        }
      }
    }   
    
  }
  
  #ifdef LEDPIN_ACTIVE
  // Turn out ACTIVE LED.
  digitalWrite(LEDPIN_ACTIVE, 0);
  #endif

  #if SHOWSTATS >= 1
  unsigned int interruptDuration = (micros2() - currentInterruptTime) & 0x7fff;   // POINTB
  
  // We are measuring the elapsed time between POINTA and POINTB in the code.
  // Assume that there are about 25 cycles of instructions in this function that are not measured, and that
  // the prologue in dispatching the function (saving registers etc) is around 51 cycles and the epilogue
  // (restoring registers etc) is around 35 cycles.  This adds a further (51+25+35)/16MHz=6.9us to the calculation.
  // See https://billgrundmann.wordpress.com/2009/03/02/the-overhead-of-arduino-interrupts/.
  interruptDuration += 7;
  
  // Record result
  if (interruptDuration > activeStats.maxInterruptTime) activeStats.maxInterruptTime = interruptDuration;
  if (interruptDuration < activeStats.minInterruptTime) activeStats.minInterruptTime = interruptDuration;
  activeStats.totalInterruptTime += interruptDuration;
  #endif

}

// micros2_cap() returns the value of the input capture register ICRn, i.e. the value of the 
// timer counter register TCNTn at the time the last digital input state change was detected.
// This is more accurate than reading TCNTn within the interrupt routine, as the latter depends
// on the time taken to dispatch the interrupt routine, which may be delayed by other interrupts.
// To be called only from interrupt routine.  Returns a value between 0 and 32767, so
// ensure that subtractions using this function are performed modulo 32767.
unsigned int micros2_cap() {
  #ifdef USETIMER
  unsigned int temp = ICRn;
  return temp / 2;  // Convert to microseconds
  #else
  return micros();
  #endif
}

// micros2() returns the current value of the timer counter register TCNTn.  This was 
// preconfigured to increment every 0.5ms, so the value is adjusted to milliseconds.
// To be called only from interrupt routine.  Returns a value between 0 and 32767, so
// ensure that subtractions using this function are performed modulo 32767.
unsigned int micros2() {
  #ifdef USETIMER
  unsigned int temp = TCNTn;
  return temp / 2;  // Convert to microseconds
  #else
  return micros();
  #endif
}

//=======================================================================
// Connect the scan routine to the interrupt.  It will execute on
// all changes (0->1 and 1->0).

void beginBitDetection() {
  #ifdef USETIMER
  TIMSK1 = (1 << ICIE1); // Input capture interrupt enable
  #else
  attachInterrupt(digitalPinToInterrupt(INPUTPIN), scan, CHANGE);
  #endif
}

//=======================================================================
// PrintByte prints one byte of data in hex with leading zero
// when necessary.

void printByte(byte x) {
  byte mask = 0x80;
  while(mask != 0) {
    if ((x & mask) != 0) 
      Serial.print('1');
    else
      Serial.print('0');  
    mask >>= 1;
  }
}

//=======================================================================
// PrintPacket prints the raw DCC packet contents to the 
// USB serial connection.

void printPacket(int index) {
  Serial.print(' ');
  for (byte n=1; n<dccPacket[index][0]; n++) {
    Serial.print(' ');
    printByte(dccPacket[index][n]);
  }
  Serial.println(' ');
}

//=======================================================================
// OutputStatistics writes the last set of statistics to the serial stream

void OutputStatistics() {

  if (showDiagnostics) {
    
#if SHOWSTATS >= 2
    // These counts are for half-bits, so divide by two.
    Serial.print(F("Bit Count="));
    Serial.print(lastStats.count/2);
    Serial.print(F(" (Zeros="));
    Serial.print(lastStats.count0/2);
    Serial.print(F(", Ones="));
    Serial.print(lastStats.count1/2);
    Serial.print(F("), Glitches="));
    Serial.println(lastStats.glitchCount);
#endif

#if SHOWSTATS >= 3
    Serial.print(F("Packets received="));
    Serial.print(lastStats.packetCount);
    Serial.print(F(", Checksum Error="));
    Serial.print(lastStats.checksumError);
    Serial.print(F(", Lost pkts="));
    Serial.print(lastStats.countLostPackets);
    Serial.print(F(", Long pkts="));
    Serial.println(lastStats.countLongPackets);
#endif

#if SHOWSTATS >= 1
    Serial.print(F("0 half-bit length (us): "));
    if (lastStats.min0 <= lastStats.max0) {
      Serial.print((float)lastStats.total0/lastStats.count0,1);
      Serial.print(F(" ("));
      Serial.print(lastStats.min0);
      Serial.print(F("-"));
      Serial.print(lastStats.max0);
      Serial.print(F(")"));
    } else
      Serial.print(F("<none>"));
    Serial.print(F(", 1 half-bit length (us): "));
    if (lastStats.min1 <= lastStats.max1) {
      Serial.print((float)lastStats.total1/lastStats.count1,1);
      Serial.print(F(" ("));
      Serial.print(lastStats.min1);
      Serial.print(F("-"));
      Serial.print(lastStats.max1);
      Serial.print(F(")"));
    } else
      Serial.print(F("<none>"));
    Serial.println();
    Serial.print(F("IRC Duration (us): "));
    if (lastStats.minInterruptTime <= lastStats.maxInterruptTime) {
      Serial.print((float)lastStats.totalInterruptTime/lastStats.count,1);
      Serial.print(F(" ("));
      Serial.print(lastStats.minInterruptTime);
      Serial.print(F("-"));
      Serial.print(lastStats.maxInterruptTime);
      Serial.print(F(")"));
    } else 
      Serial.print(F("<none>"));
      
    // Calculate cpu load
    unsigned long spareLoopCountPerSec = lastStats.spareLoopCount / refreshTime;
    Serial.print(F(",  CPU load: "));
    Serial.print(100 - spareLoopCountPerSec  / (maxSpareLoopCountPerSec / 100));
    Serial.print(F("%"));
    Serial.println();
    
    if (showBitLengths) {
      Serial.println(F("------ Half-bit count by length (us) -------"));
      for (int i=minBitLength; i<=maxBitLength; i++) {
        unsigned long c0 = lastStats.countByLength[0][i-minBitLength];
        unsigned long c1 = lastStats.countByLength[1][i-minBitLength];        
        if (c0 > 0 || c1 > 0) {
          Serial.print(i);
          Serial.print('\t');
          Serial.print(c0);
          Serial.print('\t');
          Serial.println(c1);
        }
      }
      Serial.println(F("--------------------------------------------"));
    }
#endif
  }
}

//=======================================================================
// ClearDCCData clears the contents of the packetHashList array.
// This array normally contains the checksums of received DCC packets, 
// and is used to suppress the decoding of repeated packets.

void ClearDccData() {
  for (byte n=0; n<packetHashListSize; n++) packetHashList[n]=0;
  bufferCounter=0;

#ifdef SHOWSTATS    
  // Copy and reset active stats.  Disabling interrupts would improve
  // consistency a bit but causes bits to be lost (CRC errors recorded).
  // Using memcpy and memset minimises the time at risk of inconsistency.
  memcpy(&lastStats, (void *)&activeStats, sizeof(struct stats));
  memset((void *)&activeStats, 0, sizeof(struct stats));
  activeStats.minInterruptTime = activeStats.min0 = activeStats.min1 = 65535;
#endif
}

//=======================================================================
// UpdateLED is called in the main loop to set/reset the LED fault indication
// in the event of a fault being detected within the sample period.

void UpdateLED() {
  static bool ledLit = false;
  if (activeStats.glitchCount > 0 || activeStats.checksumError > 0 || 
      activeStats.countLongPackets > 0 || activeStats.countLostPackets > 0) {
    if (!ledLit) {
      digitalWrite(LEDPIN_FAULT, 1);
      ledLit = true;
    }
  } else {
    if (ledLit) {
      digitalWrite(LEDPIN_FAULT, 0);
      ledLit = false;
    }
  }
}


//=======================================================================
// Validate received packet and pass to decoder.
// Return false if nothing done.

bool processDCC() {
  
  if (!packetsPending) {
    return false;
  }
  
  pktByteCount = dccPacket[inputPacket][0];
  // Check packet isn't empty
  if (pktByteCount > 0) {
    // Calculate and verify checksum
    byte checksum = 0;
    for (byte n = 1; n <= pktByteCount; n++) 
      checksum ^= dccPacket[inputPacket][n];
    if (checksum) {  // Result should be zero, if not it's an error!
#ifdef SHOWSTATS
      activeStats.checksumError++;
#endif
    } else { 
      // There is a new packet with a correct checksum
      #ifdef LEDPIN_DECODING
      digitalWrite(LEDPIN_DECODING, 1);
      #endif
      
      // Generate a cyclic hash based on the packet contents for checking if we've seen a similar packet before.
      isDifferentPacket=true;
      unsigned int hash = dccPacket[inputPacket][pktByteCount];  // calculate checksum
      for (byte n=1; n<pktByteCount; n++)
        hash = ((hash << 5) | (hash >> 11)) ^ dccPacket[inputPacket][n];
        
      // Check if packet's checksum is already in the list. 
      for (byte n=0; n<packetHashListSize ; n++) {
        if (hash==packetHashList[n]) 
          isDifferentPacket=false; 
      }
  
      if (isDifferentPacket) {
        packetHashList[bufferCounter++] = hash; // add new packet's hash to the list
        if (bufferCounter >= packetHashListSize) bufferCounter = 0;
        
        DecodePacket(inputPacket, isDifferentPacket);
      }

      // Optional test led whose brightness depends on loco speed setting.
      #ifdef LEDPIN_LOCOSPEED
      // Output to LED
      if (dccPacket[inputPacket][1]==B00000011 && dccPacket[inputPacket][2] == B00111111) { 
        analogWrite(LEDPIN_LOCOSPEED, map(dccPacket[inputPacket][3] & B01111111, 0, 127, 0, 255));
      }
      #endif

      #ifdef LEDPIN_DECODING
      digitalWrite(LEDPIN_DECODING, 0);
      #endif
    }
  }
  packetsPending--;  // Free packet buffer.
  if (++inputPacket >= nPackets) inputPacket = 0;

  return true;
}

    
//=======================================================================
// Read data from the dccPacket structure and decode into 
// textual representation.  Send results out over the USB serial
// connection.

void DecodePacket(int inputPacket, bool isDifferentPacket) {
  byte speed;

  // First determine the decoder type and address.
  if (dccPacket[inputPacket][1]==B11111111) { //Idle packet
    if (isDifferentPacket)
      Serial.println(F("Idle "));
    decoderType = 255;
  } else if (!bitRead(dccPacket[inputPacket][1],7)) { //bit7=0 -> Loc Decoder Short Address
    decoderAddress = dccPacket[inputPacket][1];
    instrByte1 = dccPacket[inputPacket][2];
    decoderType = 0;
  }
  else {
    if (bitRead(dccPacket[inputPacket][1],6)) { //bit7=1 AND bit6=1 -> Loc Decoder Long Address
      decoderAddress = 256 * (dccPacket[inputPacket][1] & B00111111) + dccPacket[inputPacket][2];
      instrByte1 = dccPacket[inputPacket][3];
      decoderType = 0;
    }
    else { //bit7=1 AND bit6=0 -> Accessory Decoder
      decoderAddress = dccPacket[inputPacket][1]&B00111111;
      instrByte1 = dccPacket[inputPacket][2];
      decoderType = 1;
    }
  }

  // Handle decoder type 0 and 1 separately.
  if (decoderType == 1) { // Accessory Basic
    if (showAcc) {
      if (instrByte1 & B10000000) { // Basic Accessory
        decoderAddress = (((~instrByte1) & B01110000) << 2) + decoderAddress;
        byte port = (instrByte1&B00000110)>>1;
        Serial.print(F("Acc "));
        Serial.print((decoderAddress-1)*4 + port + 1);
        Serial.print(' ');
        Serial.print(decoderAddress);
        Serial.print(F(":"));
        Serial.print(port);
        Serial.print(' ');
        Serial.print(bitRead(instrByte1,3));
        if (bitRead(instrByte1,0)) 
          Serial.print(F(" On"));
        else 
          Serial.print(F(" Off"));
      } else { // Accessory Extended NMRA spec is not clear about address and instruction format !!!
        Serial.print(F("Acc Ext "));
        decoderAddress = (decoderAddress << 5) + ((instrByte1 & B01110000) >> 2) + ((instrByte1 & B00000110) >> 1);
        Serial.print(decoderAddress);
        Serial.print(F(" Asp "));
        Serial.print(dccPacket[inputPacket][3],BIN);
      }
      printPacket(inputPacket);
    }
  }
  else if (decoderType == 0)  { // Loco / Multi Function Decoder
    if (showLoc && isDifferentPacket) {
      Serial.print(F("Loc "));
      Serial.print(decoderAddress);
      byte instructionType = instrByte1 >> 5;
      switch (instructionType) {

        case 0:
          Serial.print(F(" Control "));
        break;

        case 1: // Advanced Operations
          if (instrByte1==B00111111) { //128 speed steps
            if (bitRead(dccPacket[inputPacket][pktByteCount-1], 7)) 
              Serial.print(F(" Forw128 "));
            else 
              Serial.print(F(" Rev128 "));
            byte speed = dccPacket[inputPacket][pktByteCount-1] & B01111111;
            if (!speed) 
              Serial.print(F(" Stop "));
            else if (speed==1) 
              Serial.print(F(" E-stop "));
            else 
              Serial.print(speed-1);
          } else if (instrByte1==B00111110) { //Speed Restriction
            if (bitRead(dccPacket[inputPacket][pktByteCount-1], 7)) 
              Serial.print(F(" On "));
            else 
              Serial.print(F(" Off "));
            Serial.print(dccPacket[inputPacket][pktByteCount-1] & B01111111);
          }
        break;

        case 2: // Reverse speed step
          speed = ((instrByte1 & B00001111) << 1) - 3 + bitRead(instrByte1,4);
          if (speed==253 || speed==254) 
            Serial.print(F(" Stop "));
          else if (speed==255 || speed==0) 
            Serial.print(F(" E-Stop "));
          else {
            Serial.print(F(" Rev "));
            Serial.print(speed);
          }
        break;

        case 3: // Forward speed step
          speed = ((instrByte1 & B00001111) << 1) - 3 + bitRead(instrByte1,4);
          if (speed==253 || speed==254) 
            Serial.print(F(" Stop "));
          else if (speed==255 || speed==0) 
            Serial.print(F(" E-Stop "));
          else {
            Serial.print(F(" Forw "));
            Serial.print(speed);
          }
        break;

        case 4: // Loc Function L-4-3-2-1
          Serial.print(F(" L F4-F1 "));
          Serial.print(instrByte1 & B00011111, BIN);
        break;

        case 5: // Loc Function 8-7-6-5
          if (bitRead(instrByte1,4)) {
            Serial.print(F(" F8-F5 "));
            Serial.print(instrByte1 & B00001111, BIN);
          }
          else { // Loc Function 12-11-10-9
            Serial.print(F(" F12-F9 "));
            Serial.print(instrByte1 & B00001111, BIN);
          }
        break;

        default: 
          Serial.print(F(" unknown"));
        break;

        case 6: // Future Expansions
          switch (instrByte1&B00011111) {
            case 0: // Binary State Control Instruction long form
              Serial.print(F(" BinStateLong "));
              Serial.print(256 * dccPacket[inputPacket][pktByteCount-1] + (dccPacket[inputPacket][pktByteCount-2] & B01111111));
              if bitRead(dccPacket[inputPacket][pktByteCount-2], 7) Serial.print(F(" On "));
              else Serial.print(F(" Off "));
            break;
            case B00011101: // Binary State Control
              Serial.print(F(" BinStateShort "));
              Serial.print(dccPacket[inputPacket][pktByteCount-1] & B01111111);
              if bitRead(dccPacket[inputPacket][pktByteCount-1], 7) Serial.print(F(" On "));
              else Serial.print(F(" Off "));
            break;
            case B00011110: // F13-F20 Function Control
              Serial.print(F(" F20-F13 "));
              Serial.print(dccPacket[inputPacket][pktByteCount-1], BIN);
            break;
            case B00011111: // F21-F28 Function Control
              Serial.print(F(" F28-F21 "));
              Serial.print(dccPacket[inputPacket][pktByteCount-1], BIN);
            break;
            default:
              Serial.print(F(" unknown"));
            break;
          }
        break;

        case 7:
          Serial.print(F(" CV "));
          byte value = dccPacket[inputPacket][pktByteCount-1];
          if (instrByte1&B00010000) { // CV Short Form
            byte cvType=instrByte1 & B00001111;
            switch (cvType) {
              case B00000010:
                Serial.print(F("23 "));
                Serial.print(value);
              break;
              case B00000011:
                Serial.print(F("24 "));
                Serial.print(value);
              break;
              case B00001001:
                Serial.print(F("Decoder Lock "));
                Serial.print(value);
              break;
              default:
                Serial.print(F("unknown"));
                Serial.print(' ');
                Serial.print(value);
              break;
            }
          }
          else { // CV Long Form
            int cvAddress = 256 * (instrByte1 & B00000011) + dccPacket[inputPacket][pktByteCount-2] + 1;
            Serial.print(cvAddress);
            Serial.print(' ');
            switch (instrByte1 & B00001100) {
              case B00000100: // Verify Byte
                Serial.print(F("Verify "));
                Serial.print(value);
              break;
              case B00001100: // Write Byte
                Serial.print(F("Write "));
                Serial.print(value);
              break;
              case B00001000: // Bit Write
                Serial.print(F("Bit "));
                if (value & B00010000) Serial.print(F("Verify "));
                else Serial.print(F("Write "));
                Serial.print(value & B00000111);
                Serial.print(' ');
                Serial.print((value & B00001000)>>3);
              break;
              default:
                Serial.print(F("unknown"));
              break;
            }
          }
        break;
      }
      printPacket(inputPacket);
    }
  }
}

//=======================================================================
// Process commands sent over the USB serial connection.
//  Return false if nothing done.

bool processCommands() {
  if (Serial.available()) {
    switch (Serial.read()) {
      case 49: 
        Serial.println(F("Refresh Time = 1s"));
        refreshTime=1;
      break;
      case 50:
        Serial.println(F("Refresh Time = 2s"));
        refreshTime=2;
      break;
      case 51:
        Serial.println(F("Refresh Time = 4s"));
        refreshTime=4;
      break;
      case 52:
        Serial.println(F("Refresh Time = 8s"));
        refreshTime=8;
      break;
      case 53:
        Serial.println(F("Refresh Time = 16s"));
        refreshTime=16;
      break;
      case 54:
        Serial.println(F("Buffer Size = 4"));
        packetHashListSize=2;
      break;
      case 55:
        Serial.println(F("Buffer Size = 8"));
        packetHashListSize=8;
      break;
      case 56:
        Serial.println(F("Buffer Size = 16"));
        packetHashListSize=16;
      break;
      case 57:
        Serial.println(F("Buffer Size = 32"));
        packetHashListSize=32;
      break;
      case 48:
        Serial.println(F("Buffer Size = 64"));
        packetHashListSize=64;
      break;
      case 'a': case 'A':
        showAcc=!showAcc;
        Serial.print(F("show accessory packets = "));
        Serial.println(showAcc);
      break;
      case 'l': case 'L':
        showLoc=!showLoc;
        Serial.print(F("show loco packets = "));
        Serial.println(showLoc);
      break;
      case 'h': case 'H':
        showHeartBeat = !showHeartBeat;
        Serial.print(F("show heartbeat = "));
        Serial.println(showHeartBeat);
      break;
      case 'd': case 'D':
        showDiagnostics = !showDiagnostics;
        Serial.print(F("show diagnostics = "));
        Serial.println(showDiagnostics);
      break;
      case 'f': case 'F':
        filterInput = !filterInput;
        Serial.print(F("filter input = "));
        Serial.println(filterInput);
        break;
      case 'b': case 'B':
        showBitLengths = !showBitLengths;
        Serial.print(F("show bit lengths = "));
        Serial.println(showBitLengths);
        break;
      case '?': 
        Serial.println();
        Serial.println(F("Keyboard commands that can be sent via Serial Monitor:"));
        Serial.println(F("1 = 1s refresh time"));
        Serial.println(F("2 = 2s"));
        Serial.println(F("3 = 4s (default)"));
        Serial.println(F("4 = 8s"));
        Serial.println(F("5 = 16s"));
        Serial.println(F("6 = 4 DCC packet buffer"));
        Serial.println(F("7 = 8"));
        Serial.println(F("8 = 16"));
        Serial.println(F("9 = 32 (default)"));
       Serial.println(F("0 = 64"));
        Serial.println(F("a = show accessory packets toggle"));
        Serial.println(F("l = show locomotive packets toggle"));
  #if SHOWSTATS > 0
        Serial.println(F("d = show diagnostics toggle"));
  #endif
        Serial.println(F("h = show heartbeat toggle"));
        Serial.println(F("f = filter input toggle"));
        Serial.println(F("b = bitlength distribution"));
        Serial.println(F("? = help (show this information)"));
        Serial.print(F("ShowLoco "));
        Serial.print(showLoc);
        Serial.print(F(" / ShowAcc "));
        Serial.print(showAcc);
        Serial.print(F(" / RefreshTime "));
        Serial.print(refreshTime);
        Serial.print(F(" / BufferSize "));
        Serial.println(packetHashListSize);
        Serial.println();
      break;
    }
    return true;
  } else
    return false;
}
