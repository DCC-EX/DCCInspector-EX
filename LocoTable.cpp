#include "LocoTable.h"

LocoTable::LOCO LocoTable::speedTable[MAX_LOCOS];
int LocoTable::highestUsedReg = 0;

int LocoTable::lookupSpeedTable(int locoId, bool autoCreate) {
  // determine speed reg for this loco
  int firstEmpty = MAX_LOCOS;
  int reg;
  for (reg = 0; reg < MAX_LOCOS; reg++) {
    if (speedTable[reg].loco == locoId) break;
    if (speedTable[reg].loco == 0 && firstEmpty == MAX_LOCOS) firstEmpty = reg;
  }

  // return -1 if not found and not auto creating
  if (reg == MAX_LOCOS && !autoCreate) return -1; 
  if (reg == MAX_LOCOS) reg = firstEmpty;
  if (reg >= MAX_LOCOS) {
    //DIAG(F("Too many locos"));
    return -1;
  }
  if (reg==firstEmpty){
        speedTable[reg].loco = locoId;
        speedTable[reg].speedCode=128;  // default direction forward
        speedTable[reg].groupFlags=0;
        speedTable[reg].functions=0;
  }
  if (reg > highestUsedReg) highestUsedReg = reg;
  return reg;
}

// returns false only if loco existed but nothing was changed
bool LocoTable::updateLocoReminder(int loco, byte speedCode) {
  if (loco==0) {
    /*
     // broadcast stop/estop but dont change direction
     for (int reg = 0; reg < highestUsedReg; reg++) {
       if (speedTable[reg].loco==0) continue;
       byte newspeed=(speedTable[reg].speedCode & 0x80) |  (speedCode & 0x7f);
       if (speedTable[reg].speedCode != newspeed) {
         speedTable[reg].speedCode = newspeed;
         CommandDistributor::broadcastLoco(reg);
       }
     }
    */
     return true;
  }

  // determine speed reg for this loco
  int reg=lookupSpeedTable(loco, false);
  if (reg>=0) {
    if (speedTable[reg].speedCode!=speedCode) {
      speedTable[reg].speedCode = speedCode;
      return true;
    } else {
      return false;
    }
  } else {
    // new
    reg=lookupSpeedTable(loco, true);
    if(reg >=0) speedTable[reg].speedCode = speedCode;
    return true;
  }
}

bool LocoTable::updateFunc(int loco, byte func, int shift) {
  unsigned long previous;
  unsigned long newfunc;
  bool retval = false; // nothing was touched
  int reg = lookupSpeedTable(loco, false);
  if (reg < 0) { // not found
    retval = true;
    reg = lookupSpeedTable(loco, true);
    newfunc = previous = 0;
  } else {
    newfunc = previous = speedTable[reg].functions;
  }
    
  if(shift == 0) { // special case for light
    newfunc &= ~1UL;
    newfunc |= ((func & 0B10000) >> 4);
  } else {
    newfunc &= ~(0B1111UL << shift);
    newfunc |=  ((func & 0B1111) << shift);
  }

  if (newfunc != previous) {
    speedTable[reg].functions = newfunc;
    retval = true;
  }
  return retval;
}

