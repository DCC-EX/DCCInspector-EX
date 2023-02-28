#include <Arduino.h>

#define MAX_LOCOS 128

class LocoTable {
public:
  void forgetLoco(int cab) {
    int reg=lookupSpeedTable(cab, false);
    if (reg>=0) speedTable[reg].loco=0;
  }
  static int lookupSpeedTable(int locoId, bool autoCreate);
  static bool updateLoco(int loco, byte speedCode);
  static bool updateFunc(int loco, byte func, int shift);

private:
  struct LOCO
  {
    int loco;
    byte speedCode;
    byte groupFlags;
    unsigned long functions;
  };
  static LOCO speedTable[MAX_LOCOS];
  static int highestUsedReg;
};
