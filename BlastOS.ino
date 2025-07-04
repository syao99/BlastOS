/*

BlastOS v0.1 for brushless+solenoid flywheel blasters, based off TagBot Revision 000
By m0useCat

Table of Contents (use ctrl-f):
1. Pinout
2. Data Types
3. Global State & Defaults
4. Core Methods
5. setup() and loop()

*/
#define CTWI_USING_colING_ACCESS

#include <Servo.h>  // PWM output to ESC's
#include <nI2C.h>   // Screen
//#include <EEPROM.h> // Save/load profiles

/*
todo:
switch pusher to pin 10 after debugging.
implement UI layer
implement controls layer
*/

// 1. Pinout
// Input
#define PINREV 11
#define PINTRIG 12
#define PINSELECT1 A0
#define PINSELECT2 A1
#define PINMENU A2
#define PINVOLTAGE A3
// Output
#define PINESC 8
#define PINPUSHER 13  //10 USE PIN 10 FOR IMPLEMENTATION, 13 IS FOR LED DEBUGGING. PIN 13 WILL ACTUATE THE NOID ON POWERUP, POSSIBLY CAUSING JAMS.
//#define PINPUSHER 10
// Other Params
#define WHEELMINSPEED 1000
#define WHEELMAXSPEED 2000
#define MAXVOLTAGE 16.8
#define DIVIDEDVOLTAGE 5
#define ENABLEDECAY true

#define SCREEN_ADDR 0x3C
#define ROW_LAST 7
#define COL_LAST 15
#define ROW_COUNT 8
#define COL_COUNT 16

const char versionText[] = "BlastOS v0.1";

//#define EEPROMOFFSET 24  // max 1000, add 24 if EEPROM issues occur. 24 may actually need to be a different value depending on save config data

// 2. Types
enum class UseBootMode : uint8_t { BOOTMID,
                                   BOOTFRONT,
                                   BOOTBACK,
                                   BOOTCONFIG };  // zero: default on tb2, middle position phanta.
//maybe refactor this to just uint8_t, zero for config. idk yet.

struct Debounceable {
  const uint8_t debounceTime;
  unsigned long debounceStartTime = 0;
  Debounceable(uint8_t time = 10)
    : debounceTime(time) {}
  void resetDebounce() {
    debounceStartTime = millis();
  }
  bool isDebounced() {
    return (millis() - debounceStartTime) >= debounceTime;
  }
};
struct GlobalParams {
  uint8_t maxDPS = 20;
  uint8_t noidOnTime = 25;
  uint8_t compLockProfile = 0;  // 0 disabled, 1/2/3 for corresponding profiles
  uint16_t voltageThreshold = 144;
  float decayMultiplier = 0.99f;
  GlobalParams() {}
};
struct ProfileParams {
  uint8_t noidDPS;
  float fracVelMultiplier;
  uint8_t firingMode;  // 0: safe, 1: semi, 2-254: burst, 255: auto
  //bool leftyMode = false;
  ProfileParams(
    uint8_t DPS = 12,
    float fvMultiplier = 0.5f,
    uint8_t mode = 3  //,
    //bool left = false
    )
    : noidDPS(DPS),
      fracVelMultiplier(fvMultiplier),
      firingMode(mode) {}
};
struct GlobalState {
  uint16_t targetVelocity = 1200;
  unsigned long cycleStartTime = 0;
  bool isCycleActive = false;
  bool previousIsFiring = false;
  //bool previousIsTriggerPulled = false;
  bool previousIsMenuPressed = false;
  uint8_t burstCounter = 0;
  bool noidState = false;
  uint16_t currentRevSpeed = WHEELMINSPEED;
  uint16_t noidOffTime = 30;
  uint8_t currentFiringProfileIndex = 0;
  uint8_t previousFiringProfileIndex = 0;
  UseBootMode useMode;
  char* bootModeText;
  GlobalState() {}
};
struct Haptic {  // for haptics
  unsigned long startTime = 0;
  bool isActive = false;  // external control to enable or shut off the cycle. takes effect immediately but can fix that later.
  bool state = false;
  const uint8_t cycleCount;
  const uint16_t onTime;
  const uint16_t offTime;
  uint16_t cycleTime;
  uint16_t duration;
  unsigned long elapsed;
  Haptic(uint8_t count = 3, uint16_t on = 6, uint16_t off = 6)
    : cycleCount(count),
      onTime(on),
      offTime(off) {}
  void start() {
    if (isActive) return;
    startTime = millis();
    isActive = true;
    cycleTime = onTime + offTime;
    duration = cycleTime * cycleCount;
  }
  bool isDone() {
    return elapsed > duration;
  }
  bool getUpdatedStatus() {
    if (!isActive) return false;
    elapsed = millis() - startTime;
    if (isDone()) {
      //stop();
      return false;
    }
    state = (elapsed % cycleTime) < onTime;
    return state;
    // verify logical premise in js browser console:
    //for (i = 0; i<100; i++) {
    //  console.log((i % 10)<4);
    //}
  }
  void reset() {
    isActive = false;
  }
};
struct RateLimitedAction {  // rate limited action. use requestExec() to call, then if (shouldExec()) function() to call.
  const uint8_t delay;
  unsigned long startTime;
  bool isActive = false;
  bool shouldExec = false;
  RateLimitedAction(uint8_t timing = 100)
    : delay(timing) {}
  void requestExec() {
    if (isActive) return;
    startTime = millis();
    isActive = true;
    shouldExec = true;
  }
  bool findShouldExec() {
    if (!isActive) return false;
    if (shouldExec) {
      shouldExec = false;
      return true;
    }
    if ((millis() - startTime) > delay) {
      isActive = false;
      return false;
    }
  }
};
/*struct SingleAction {
  bool isActive = false;
  bool shouldExec = false;
  SingleAction() {}
  void requestExec() {
    if (isActive) return;
    isActive = true;
    shouldExec = true;
  }
  bool findShouldExec() {
    if (!isActive) return false;
    if (shouldExec) {
      shouldExec = false;
      return true;
    }
  }
};*/
struct SingleAction {
  bool isLocked = false;
  SingleAction() {}
  bool requestExec() {
    if (isLocked) return false;
    isLocked = true;
    return true;
  }
  void reset() {
    isLocked = false;
  }
};

const uint8_t spriteSheet[128][8] PROGMEM = {
  [0] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    // ' '
  [1] = { 0x00, 0x7c, 0xfe, 0x92, 0x8a, 0xfe, 0x7c, 0x00 },    // '0'
  [2] = { 0x00, 0x00, 0x02, 0xfe, 0xfe, 0x42, 0x00, 0x00 },    // '1'
  [3] = { 0x00, 0x66, 0xf6, 0x92, 0x9a, 0xce, 0x46, 0x00 },    // '2'
  [4] = { 0x00, 0x6c, 0xfe, 0x92, 0x92, 0xc6, 0x44, 0x00 },    // '3'
  [5] = { 0x00, 0x0a, 0xfe, 0xfe, 0x4a, 0x28, 0x18, 0x00 },    // '4'
  [6] = { 0x00, 0x9c, 0xbe, 0xa2, 0xa2, 0xe6, 0xe4, 0x00 },    // '5'
  [7] = { 0x00, 0x0c, 0x9e, 0x92, 0xd2, 0x7e, 0x3c, 0x00 },    // '6'
  [8] = { 0x00, 0xc0, 0xe0, 0xb0, 0x9e, 0x8e, 0x80, 0x00 },    // '7'
  [9] = { 0x00, 0x6c, 0xfe, 0x92, 0x92, 0xfe, 0x6c, 0x00 },    // '8'
  [10] = { 0x00, 0x78, 0xfc, 0x96, 0x92, 0xf2, 0x60, 0x00 },   // '9'
  [11] = { 0x00, 0x3e, 0x7e, 0xc8, 0xc8, 0x7e, 0x3e, 0x00 },   // 'A'
  [12] = { 0x00, 0x6c, 0xfe, 0x92, 0x92, 0xfe, 0xfe, 0x00 },   // 'B'
  [13] = { 0x00, 0x44, 0xc6, 0x82, 0x82, 0xfe, 0x7c, 0x00 },   // 'C'
  [14] = { 0x00, 0x38, 0x7c, 0xc6, 0x82, 0xfe, 0xfe, 0x00 },   // 'D'
  [15] = { 0x00, 0x82, 0x92, 0x92, 0x92, 0xfe, 0xfe, 0x00 },   // 'E'
  [16] = { 0x00, 0x80, 0x90, 0x90, 0x90, 0xfe, 0xfe, 0x00 },   // 'F'
  [17] = { 0x00, 0x5c, 0xde, 0x92, 0x82, 0xfe, 0x7c, 0x00 },   // 'G'
  [18] = { 0x00, 0xfe, 0xfe, 0x10, 0x10, 0xfe, 0xfe, 0x00 },   // 'H'
  [19] = { 0x00, 0x00, 0x82, 0xfe, 0xfe, 0x82, 0x00, 0x00 },   // 'I'
  [20] = { 0x00, 0x80, 0xfc, 0xfe, 0x82, 0x06, 0x04, 0x00 },   // 'J'
  [21] = { 0x00, 0x82, 0xc6, 0x6c, 0x38, 0xfe, 0xfe, 0x00 },   // 'K'
  [22] = { 0x00, 0x02, 0x02, 0x02, 0x02, 0xfe, 0xfe, 0x00 },   // 'L'
  [23] = { 0xfe, 0xfe, 0x60, 0x30, 0x60, 0xfe, 0xfe, 0x00 },   // 'M'
  [24] = { 0x00, 0xfe, 0xfe, 0x30, 0x60, 0xfe, 0xfe, 0x00 },   // 'N'
  [25] = { 0x00, 0x7c, 0xfe, 0x82, 0x82, 0xfe, 0x7c, 0x00 },   // 'O'
  [26] = { 0x00, 0x60, 0xf0, 0x90, 0x90, 0xfe, 0xfe, 0x00 },   // 'P'
  [27] = { 0x00, 0x7a, 0xfe, 0x8c, 0x82, 0xfe, 0x7c, 0x00 },   // 'Q'
  [28] = { 0x00, 0x62, 0xf6, 0x9c, 0x98, 0xfe, 0xfe, 0x00 },   // 'R'
  [29] = { 0x00, 0x4c, 0xde, 0x92, 0x92, 0xf6, 0x64, 0x00 },   // 'S'
  [30] = { 0x00, 0xc0, 0x82, 0xfe, 0xfe, 0x82, 0xc0, 0x00 },   // 'T'
  [31] = { 0x00, 0xfc, 0xfe, 0x02, 0x02, 0xfe, 0xfc, 0x00 },   // 'U'
  [32] = { 0x00, 0xf8, 0xfc, 0x06, 0x06, 0xfc, 0xf8, 0x00 },   // 'V'
  [33] = { 0xfe, 0xfe, 0x0c, 0x18, 0x0c, 0xfe, 0xfe, 0x00 },   // 'W'
  [34] = { 0x00, 0xc6, 0xee, 0x38, 0x38, 0xee, 0xc6, 0x00 },   // 'X'
  [35] = { 0x00, 0xe0, 0xf2, 0x1e, 0x1e, 0xf2, 0xe0, 0x00 },   // 'Y'
  [36] = { 0x00, 0xc2, 0xe2, 0xb2, 0x9a, 0x8e, 0x86, 0x00 },   // 'Z'
  [37] = { 0x02, 0x1e, 0x3c, 0x2a, 0x2a, 0x2e, 0x04, 0x00 },   // 'a'
  [38] = { 0x00, 0x1c, 0x3e, 0x22, 0x22, 0xfe, 0xfe, 0x00 },   // 'b'
  [39] = { 0x00, 0x14, 0x36, 0x22, 0x22, 0x3e, 0x1c, 0x00 },   // 'c'
  [40] = { 0x00, 0xfe, 0xfe, 0x22, 0x22, 0x3e, 0x1c, 0x00 },   // 'd'
  [41] = { 0x00, 0x18, 0x3a, 0x2a, 0x2a, 0x3e, 0x1c, 0x00 },   // 'e'
  [42] = { 0x00, 0x40, 0xd0, 0x90, 0xfe, 0x7e, 0x10, 0x00 },   // 'f'
  [43] = { 0x00, 0x3e, 0x3f, 0x25, 0x25, 0x3d, 0x19, 0x00 },   // 'g'
  [44] = { 0x00, 0x1e, 0x3e, 0x20, 0x20, 0xfe, 0xfe, 0x00 },   // 'h'
  [45] = { 0x00, 0x00, 0x02, 0xbe, 0xbe, 0x22, 0x00, 0x00 },   // 'i'
  [46] = { 0x00, 0xbe, 0xbf, 0x01, 0x01, 0x07, 0x06, 0x00 },   // 'j'
  [47] = { 0x00, 0x22, 0x36, 0x1c, 0x08, 0xfe, 0xfe, 0x00 },   // 'k'
  [48] = { 0x00, 0x00, 0x02, 0xfe, 0xfe, 0x82, 0x00, 0x00 },   // 'l'
  [49] = { 0x00, 0x18, 0x3e, 0x3e, 0x18, 0x3e, 0x3e, 0x00 },   // 'm'
  [50] = { 0x00, 0x1e, 0x3e, 0x20, 0x20, 0x3e, 0x3e, 0x00 },   // 'n'
  [51] = { 0x00, 0x1c, 0x3e, 0x22, 0x22, 0x3e, 0x1c, 0x00 },   // 'o'
  [52] = { 0x00, 0x18, 0x3c, 0x24, 0x24, 0x3f, 0x3f, 0x00 },   // 'p'
  [53] = { 0x00, 0x3f, 0x3f, 0x24, 0x24, 0x3c, 0x18, 0x00 },   // 'q'
  [54] = { 0x00, 0x20, 0x30, 0x30, 0x10, 0x3e, 0x3e, 0x00 },   // 'r'
  [55] = { 0x00, 0x24, 0x2e, 0x2a, 0x2a, 0x3a, 0x12, 0x00 },   // 's'
  [56] = { 0x00, 0x04, 0x26, 0x22, 0xfe, 0xfc, 0x20, 0x00 },   // 't'
  [57] = { 0x00, 0x3e, 0x3e, 0x02, 0x02, 0x3e, 0x3c, 0x00 },   // 'u'
  [58] = { 0x00, 0x38, 0x3c, 0x06, 0x06, 0x3c, 0x38, 0x00 },   // 'v'
  [59] = { 0x38, 0x3e, 0x0e, 0x18, 0x0e, 0x3e, 0x38, 0x00 },   // 'w'
  [60] = { 0x00, 0x22, 0x36, 0x1c, 0x1c, 0x36, 0x22, 0x00 },   // 'x'
  [61] = { 0x00, 0x3e, 0x3f, 0x05, 0x05, 0x3d, 0x39, 0x00 },   // 'y'
  [62] = { 0x00, 0x22, 0x32, 0x3a, 0x2e, 0x26, 0x22, 0x00 },   // 'z'
  [63] = { 0x00, 0x00, 0x60, 0xfa, 0xfa, 0x60, 0x00, 0x00 },   // '!'
  [64] = { 0x00, 0x00, 0xc0, 0xe0, 0x00, 0xc0, 0xe0, 0x00 },   // '"'
  [65] = { 0x00, 0x28, 0xfe, 0xfe, 0x28, 0xfe, 0xfe, 0x28 },   // '#'
  [66] = { 0x00, 0x08, 0x5c, 0x54, 0xfe, 0x54, 0x74, 0x24 },   // '$'
  [67] = { 0x00, 0x46, 0x66, 0x30, 0x18, 0x0c, 0x66, 0x62 },   // '%'
  [68] = { 0x00, 0x12, 0x5e, 0xec, 0xba, 0xf2, 0x5e, 0x0c },   // '&'
  [69] = { 0x00, 0x00, 0x00, 0x00, 0xc0, 0xe0, 0x20, 0x00 },   // '''
  [70] = { 0x00, 0x00, 0x82, 0xc6, 0x7c, 0x38, 0x00, 0x00 },   // '('
  [71] = { 0x00, 0x00, 0x38, 0x7c, 0xc6, 0x82, 0x00, 0x00 },   // ')'
  [72] = { 0x22, 0x36, 0x1c, 0x7f, 0x7f, 0x1c, 0x36, 0x22 },   // '*'
  [73] = { 0x00, 0x10, 0x10, 0x7c, 0x7c, 0x10, 0x10, 0x00 },   // '+'
  [74] = { 0x00, 0x00, 0x00, 0x06, 0x07, 0x01, 0x00, 0x00 },   // ','
  [75] = { 0x00, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00 },   // '-'
  [76] = { 0x00, 0x00, 0x00, 0x06, 0x06, 0x00, 0x00, 0x00 },   // '.'
  [77] = { 0x00, 0x80, 0xc0, 0x60, 0x30, 0x18, 0x0c, 0x06 },   // '/'
  [78] = { 0x00, 0x00, 0x00, 0x66, 0x66, 0x00, 0x00, 0x00 },   // ':'
  [79] = { 0x00, 0x00, 0x00, 0x66, 0x67, 0x01, 0x00, 0x00 },   // ';'
  [80] = { 0x00, 0x00, 0x82, 0xc6, 0x6c, 0x38, 0x10, 0x00 },   // '<'
  [81] = { 0x00, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x00 },   // '='
  [82] = { 0x00, 0x10, 0x38, 0x6c, 0xc6, 0x82, 0x00, 0x00 },   // '>'
  [83] = { 0x00, 0x60, 0xf0, 0x9a, 0x8a, 0xc0, 0x40, 0x00 },   // '?'
  [84] = { 0x00, 0x78, 0xfa, 0xba, 0x82, 0xfe, 0x7c, 0x00 },   // '@'
  [85] = { 0x00, 0x00, 0x82, 0x82, 0xfe, 0xfe, 0x00, 0x00 },   // '['
  [86] = { 0x02, 0x06, 0x0c, 0x18, 0x30, 0x60, 0xc0, 0x80 },   // '\'
  [87] = { 0x00, 0x00, 0xfe, 0xfe, 0x82, 0x82, 0x00, 0x00 },   // ']'
  [88] = { 0x00, 0x10, 0x30, 0x60, 0xc0, 0x60, 0x30, 0x10 },   // '^'
  [89] = { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 },   // '_'
  [90] = { 0x00, 0x00, 0x20, 0x60, 0xc0, 0x80, 0x00, 0x00 },   // '`'
  [91] = { 0x00, 0x82, 0x82, 0xee, 0x7c, 0x10, 0x10, 0x00 },   // '{'
  [92] = { 0x00, 0x00, 0x00, 0xee, 0xee, 0x00, 0x00, 0x00 },   // '|'
  [93] = { 0x00, 0x10, 0x10, 0x7c, 0xee, 0x82, 0x82, 0x00 },   // '}'
  [94] = { 0x00, 0x80, 0xc0, 0x40, 0xc0, 0x80, 0xc0, 0x40 },   // '~'
  [95] = { 0x10, 0x30, 0x70, 0xF0, 0xF0, 0x70, 0x30, 0x10 },   // Triangle UpD
  [96] = { 0x18, 0x3C, 0x7E, 0xFF, 0x00, 0x00, 0x00, 0x00 },   // Triangle RightL
  [97] = { 0x08, 0x0C, 0x0E, 0x0F, 0x0F, 0x0E, 0x0C, 0x08 },   // Triangle DownU
  [98] = { 0x00, 0x00, 0x00, 0x00, 0xFF, 0x7E, 0x3C, 0x18 },   // Triangle LeftR
  [99] = { 0x00, 0x18, 0x3C, 0x24, 0x24, 0x24, 0x24, 0x3C },   // Battery
  [100] = { 0x1C, 0x3E, 0x22, 0x22, 0x22, 0x22, 0x22, 0x3E },  // Battery2
  [101] = { 0x00, 0x18, 0x1E, 0x1F, 0xF8, 0x78, 0x18, 0x00 },  // Power
  [102] = { 0x0C, 0x10, 0x28, 0x24, 0x20, 0x20, 0x10, 0x0C },  // Speed
  [103] = { 0x00, 0x2A, 0x7F, 0x55, 0x55, 0x55, 0x7F, 0x00 },  // Darts
  [104] = { 0x1C, 0x1C, 0x22, 0x22, 0x22, 0x22, 0x22, 0x3E },  // Dart
  [105] = { 0x10, 0x28, 0x44, 0x82, 0x82, 0x44, 0x28, 0x10 },  // Diamond
  [106] = { 0x00, 0x00, 0x54, 0x54, 0x54, 0x54, 0x54, 0x00 },  // Menu Settings
};
struct ScreenMgr {
  CI2C* i2c;
  CI2C::Handle screen;

  uint8_t tileMap[8][16];

  bool setTileMapAt(uint8_t val, uint8_t row, uint8_t col) {
    if (row > ROW_LAST) return false;
    if (col > COL_LAST) return false;
    tileMap[row][col] = val;
    return true;
  }

  bool invertTileMapAt(uint8_t row, uint8_t col) {
    if (row > ROW_LAST) return false;
    if (col > COL_LAST) return false;
    tileMap[row][col] ^= 0x80;
    return true;
  }

  uint8_t getTileMapAt(uint8_t row, uint8_t col) {
    if (row > ROW_LAST) row = ROW_LAST;
    if (col > COL_LAST) col = COL_LAST;
    return tileMap[row][col];
  }
  // send one or more control+payload bytes in coling mode
  bool sendcol(uint8_t ctrl, const uint8_t* data, uint16_t len) {
    //uint16_t total = len + 1;
    if (len > (CTWI::SIZE_BUFFER - 2)) return false;
    static uint8_t buf[CTWI::SIZE_BUFFER + 1];
    buf[0] = ctrl;
    memcpy(buf + 1, data, len);
    return (i2c->Write(screen, buf, len + 1) == CI2C::STATUS_OK);
  }

  void initDisplay() {
    i2c = nI2C;
    i2c->SetTimeoutMS(100);
    screen = i2c->RegisterDevice(SCREEN_ADDR, 1, CI2C::Speed::FAST);

    const uint8_t initCmds[] = {
      0xAE,        // display off
      0xD5, 0x80,  // set display clock
      0xA8, 0x3F,  // multiplex
      0xD3, 0x00,  // display offset
      0x40,        // start line
      0x8D, 0x14,  // charge pump
      0x20, 0x00,  // memory mode
      0xA1,        // seg remap
      0xC8,        // com scan dec
      0xDA, 0x12,  // com pins
      0x81, 0xCF,  // contrast
      0xD9, 0xF1,  // pre-charge
      0xDB, 0x40,  // vcom detect
      0xA4,        // resume display
      0xA6,        // normal display
      0xAF         // display on
    };

    for (uint8_t i = 0; i < sizeof(initCmds); ++i)
      sendcol(0x00, &initCmds[i], 1);
  }

  void updateScreenFromTilemap(const uint8_t (*spriteSheet)[8]) {
    // reserve 1 byte for SSD1306 control, 1 for nI2C register address
    //const uint8_t capacity = CTWI::SIZE_BUFFER - 2;
    for (uint8_t row = 0; row < 8; ++row) {
      uint8_t cmd;

      // set row address
      cmd = 0xB0 | row;
      sendcol(0x00, &cmd, 1);
      // set column low nibble = 0
      cmd = 0x00;
      sendcol(0x00, &cmd, 1);
      // set column high nibble = 0
      cmd = 0x10;
      sendcol(0x00, &cmd, 1);

      // stream each tile’s 8 bytes directly
      for (uint8_t col = 0; col < 16; ++col) {
        //uint8_t entry = getTileMapAt(row,col); // SWITCH TO THIS TO INVERT ADDRESSES. SPRITES HAVE TO BE INVERTED SEPERATELY.
        uint8_t entry = getTileMapAt(ROW_LAST - row, COL_LAST - col);
        uint8_t id = entry & 0x7F;
        bool invert = entry & 0x80;
        for (uint8_t col = 0; col < 8; ++col) {
          uint8_t b = pgm_read_byte(&spriteSheet[id][col]);
          if (invert) b = ~b;
          sendcol(0x40, &b, 1);
        }
      }
    }
  }

  void updateScreen() {
    updateScreenFromTilemap(spriteSheet);
  }

  uint8_t charToTileID(char c) {
    if (c == ' ') return 0;
    if (c >= '0' && c <= '9') return 1 + (c - '0');   // '0'→1 … '9'→10
    if (c >= 'A' && c <= 'Z') return 11 + (c - 'A');  // 'A'→11 … 'Z'→36
    if (c >= 'a' && c <= 'z') return 37 + (c - 'a');  // 'a'→37 … 'z'→62
    switch (c) {
      case '!': return 63;
      case '"': return 64;
      case '#': return 65;
      case '$': return 66;
      case '%': return 67;
      case '&': return 68;
      case '\'': return 69;
      case '(': return 70;
      case ')': return 71;
      case '*': return 72;
      case '+': return 73;
      case ',': return 74;
      case '-': return 75;
      case '.': return 76;
      case '/': return 77;
      case ':': return 78;
      case ';': return 79;
      case '<': return 80;
      case '=': return 81;
      case '>': return 82;
      case '?': return 83;
      case '@': return 84;
      case '[': return 85;
      case '\\': return 86;
      case ']': return 87;
      case '^': return 88;
      case '_': return 89;
      case '`': return 90;
      case '{': return 91;
      case '|': return 92;
      case '}': return 93;
      case '~': return 94;
      default: return 0;
    }
  }


  void truncateText(char* text, uint8_t limit) {
    if (limit == 0) {
      text[0] = '\0';
      return;
    }
    for (uint8_t i = 0; i < limit; ++i) {
      if (!text[i]) return;
    }
    text[limit] = '\0';
  }


  // Render a text string at given row, start column:
  void setText(const char* text, uint8_t row, uint8_t startCol) {
    for (uint8_t i = 0; text[i] && startCol + i < COL_COUNT; ++i) {
      //row = constrain(row, 0, ROW_LAST);
      //uint8_t useCol = constrain(startCol + i, 0, COL_LAST);
      setTileMapAt(charToTileID(text[i]), row, startCol + i);
      //tileMap[row][useCol] = charToTileID(text[i]);
    }
  }

  void setSprite(uint8_t sprite, uint8_t row, uint8_t col) {
    //row = constrain(row, 0, ROW_LAST);
    //col = constrain(col, 0, COL_LAST);
    //tileMap[row][col] = sprite;
    setTileMapAt(sprite, row, col);
  }

  void invertAt(uint8_t row, uint8_t col, uint8_t span) {
    // clamp span so col+span ≤ 16
    if (col > COL_LAST) return;
    if (row > ROW_LAST) return;
    uint8_t maxSpan = (col + span > COL_COUNT ? COL_COUNT - col : span);
    for (uint8_t i = 0; i < maxSpan; ++i) {
      tileMap[row][col + i] ^= 0x80;
    }
  }

  void cycleSprites() {
    uint8_t idx = 0;
    for (uint8_t row = 0; row < 8; ++row) {
      for (uint8_t col = 0; col < 16; ++col) {
        //tileMap[ROW_LAST-row][COL_LAST-col] = idx++;
        if (row > ROW_LAST) break;
        if (col > COL_LAST) break;
        tileMap[row][col] = idx++;
      }
    }
  }

  void setupTileMapHelloWorld() {
    // Clear all to blank (0)
    for (uint8_t row = 0; row < 8; ++row)
      for (uint8_t tx = 0; tx < 16; ++tx)
        tileMap[row][tx] = 0;

    // "Hello World" length: 11 chars
    // Place horizontally centered on row 3 (row 3)
    // Center horizontally: (16 - 11) / 2 = 2.5 → start at tile 2 or 3
    uint8_t startX = 2;
    uint8_t row = 3;

    const char* text = "Hello World";

    // ASCII ' ' = 32; in spriteSheet indices, '0' starts at 1 for '0',
    // uppercase 'A' starts at 11, lowercase 'a' at 37
    for (uint8_t i = 0; text[i] != 0 && (startX + i) < 16; ++i) {
      char c = text[i];
      uint8_t tileId = 0;

      if (c >= '0' && c <= '9')
        tileId = 1 + (c - '0');  // digits 1–10
      else if (c >= 'A' && c <= 'Z')
        tileId = 11 + (c - 'A');  // uppercase letters
      else if (c >= 'a' && c <= 'z')
        tileId = 37 + (c - 'a');  // lowercase letters
      else if (c == ' ')
        tileId = 0;  // blank
      else if (c == '!')
        tileId = 63;  // exclamation mark example
      else
        tileId = 0;  // fallback blank

      tileMap[row][startX + i] = tileId;
    }
  }

  void setCheckers() {
    for (uint8_t row = 0; row < ROW_COUNT; ++row) {
      for (uint8_t col = 0; col < COL_COUNT; ++col) {
        tileMap[row][col] = ((row + col) & 1) ? 0x80 : 0x00;
      }
    }
  }

  void loopInvertAll(bool update = true) {
    static uint32_t lastToggle = 0;
    const uint32_t interval = 500;
    uint32_t now = millis();
    if (now - lastToggle >= interval) {
      lastToggle = now;
      for (uint8_t row = 0; row < 8; ++row) {
        for (uint8_t col = 0; col < COL_COUNT; ++col) {
          //tileMap[row][col] ^= 0x80;
          invertTileMapAt(row, col);
        }
      }
      if (update) updateScreen();
    }
  }
};
// core implementation always works on uint32_t
char* numToCharArray(uint16_t num) {
  static char buf[6];  // max "65535"+'\0'
  uint8_t pos = sizeof(buf) - 1;
  buf[pos] = '\0';
  do {
    buf[--pos] = '0' + (num % 10);
    num /= 10;
  } while (num);
  return &buf[pos];
}
/*template<typename IntType>
inline char* numToCharArray(IntType num) {
    return numToCharArray(static_cast<uint16_t>(num));
}*/

/*void concat(char *dest, const char *src) {
    while (*dest) dest++;
    while (*src) *dest++ = *src++;
    *dest = '\0';
}*/

char* concat(const char* a, const char* b) {
  size_t la = strlen(a);
  size_t lb = strlen(b);
  char* result = (char*)malloc(la + lb + 1);
  if (!result) return nullptr;
  memcpy(result, a, la);
  memcpy(result + la, b, lb + 1);  // copies terminating '\0'
  return result;
}

struct UIMgr {
  ScreenMgr& scrMgr;
  GlobalParams& globalParams;
  ProfileParams& profileParams;
  GlobalState& globalState;
  UIMgr(ScreenMgr& mgr, GlobalParams& gParams, ProfileParams& pParams, GlobalState& gState)
    : scrMgr(mgr), globalParams(gParams), profileParams(pParams), globalState(gState) {}
  initStatus(bool pushToScr = true) {
    scrMgr.setText(versionText, 0, 2);
    scrMgr.setText(globalState.bootModeText, 1, 3);
    if (pushToScr) scrMgr.updateScreen();
  }
  updateStatus(bool pushToScr = true) {
    scrMgr.setText(versionText, 0, 2);
    scrMgr.setText(globalState.bootModeText, 1, 3);

    scrMgr.setSprite(102, 3, 5);
    scrMgr.setText(numToCharArray(globalState.targetVelocity), 3, 6);

    scrMgr.setSprite(106, 4, 5);
    scrMgr.setText(getModeText(), 4, 6);

    scrMgr.setSprite(103, 5, 5);
    scrMgr.setText(getDPSText(), 5, 6);

    uint8_t voltage = getVoltage();
    //Serial.println(voltage);
    scrMgr.setSprite(101, 7, 5);
    scrMgr.setText(getVoltageText(voltage), 7, 6);

    if (pushToScr) scrMgr.updateScreen();
  }
};

// 3. Global State & Defaults
Servo esc;

// Global, Profile, and State

GlobalParams globalParams;
ProfileParams firingProfiles[] = {  // dps, fvMulti, mode i.e. 0: safe, 1: semi, 2-254: burst, 255: auto
  { ProfileParams(globalParams.maxDPS, 0.5f, 3) },
  { ProfileParams(globalParams.maxDPS, 0.5f, 1) },
  { ProfileParams(globalParams.maxDPS, 0.5f, 255) }
};
uint16_t bootVelocities[] = { 1200, 1400, 1600 };
GlobalState globalState;
Debounceable debounceableTrigger;
Debounceable debounceableMenu;
Haptic haptic = Haptic();
SingleAction btnAction = SingleAction();

ScreenMgr screenMgr;
UIMgr UI{ screenMgr, globalParams, firingProfiles, globalState };

uint8_t getSelectorIndex() {
  if (getDigitalPin(PINSELECT1)) return 1;
  if (getDigitalPin(PINSELECT2)) return 2;
  return 0;
}

char* getBootModeText(uint8_t index) {
  switch (index) {
    case 0: return "Profile 0";
    case 1: return "Profile 1";
    case 2: return "Profile 2";
  }
}

char* getModeText() {
  uint8_t mode = getCurrentFiringProfile().firingMode;
  switch (mode) {
    case 0: return "Safe  ";
    case 1: return "Semi  ";
    case 255: return "Auto  ";
    default: return concat("Burst", numToCharArray(mode));
  }
}

char* getDPSText() {
  uint8_t dps = getCurrentFiringProfile().noidDPS;
  return concat(numToCharArray(dps), "DPS");
}

uint16_t getVoltage() {
  uint16_t rawVal = analogRead(PINVOLTAGE);
  return rawVal;
  //return map(rawVal, 0, 1024);
}

char* getVoltageText(uint16_t voltage) {
  return "00.0v";
  //return numToCharArray(voltage);
}

ProfileParams getCurrentFiringProfile() {
  return firingProfiles[globalState.currentFiringProfileIndex];
}

void initBootVelocity() {
  uint8_t bootSelectorIndex = getSelectorIndex();
  globalState.targetVelocity = bootVelocities[bootSelectorIndex];
  globalState.bootModeText = getBootModeText(bootSelectorIndex);
}

void updateNoidOffTime() {
  globalState.noidOffTime = (1000.f / getCurrentFiringProfile().noidDPS) - globalParams.noidOnTime;
}

//uint16_t deprecated_noidOffTime = findNoidOffTime(deprecated_MAINDPS);



// 4. Core Methods


bool getDigitalPin(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

void setDigitalPin(uint8_t pin, bool isActive) {
  digitalWrite(pin, isActive);
}

void assignPins() {
  // Pin config
  pinMode(PINREV, INPUT_PULLUP);
  pinMode(PINTRIG, INPUT_PULLUP);
  pinMode(PINSELECT1, INPUT_PULLUP);
  pinMode(PINSELECT2, INPUT_PULLUP);
  pinMode(PINMENU, INPUT_PULLUP);
  pinMode(PINVOLTAGE, INPUT);
  pinMode(PINESC, OUTPUT);
  pinMode(PINPUSHER, OUTPUT);
}

UseBootMode getBootMode() {
  if (getDigitalPin(PINMENU)) return UseBootMode::BOOTCONFIG;
  if (getDigitalPin(PINSELECT1)) return UseBootMode::BOOTFRONT;
  if (getDigitalPin(PINSELECT2)) return UseBootMode::BOOTBACK;
  return UseBootMode::BOOTMID;
}

void initESC() {
  esc.attach(PINESC, WHEELMINSPEED, WHEELMAXSPEED);
  delay(100);
  esc.writeMicroseconds(WHEELMINSPEED);
}

//HapticController hapticController = HapticController(3, 12);

void resetCycle() {
  globalState.cycleStartTime = millis();
}

bool getCyclingLogic() {
  if (!debounceableTrigger.isDebounced()) return globalState.noidState;  // if not debounced, return previous value
  bool pusherOn = false;
  bool isTriggerPulled = getDigitalPin(PINTRIG);
  /*debounceableTrigger.isDebounced() ?
      getDigitalPin(PINTRIG) :
      globalState.previousIsTriggerPulled;
  globalState.previousIsTriggerPulled = isTriggerPulled;*/
  if (!globalState.isCycleActive) {  // no active cycle is happening, start a new one
    if (isTriggerPulled) {
      globalState.isCycleActive = true;
      pusherOn = true;
      resetCycle();
      resetBurstCounter();
    }
  } else {  // if cycle has already begun, we need to ensure it is on for the standard noid on time
    unsigned long currentCycleTime = millis() - globalState.cycleStartTime;
    if (currentCycleTime < (globalParams.noidOnTime + globalState.noidOffTime)) {  // if cycle time is within the entire cycling time frame
      if (currentCycleTime < globalParams.noidOnTime) {                            // if cycling time is within standard noid on time, keep it on.
        pusherOn = true;
      }
    } else if (!isTriggerPulled) {  // trigger released, end the cycle asap but allow it to finish to avoid jams.
      globalState.isCycleActive = false;
      //resetBurstCounter();
      debounceableTrigger.resetDebounce();
    } else if (isFullAuto()) {
      pusherOn = true;  // reset the firing cycle
      resetCycle();
    } else if (hasRemainingBursts()) {
      globalState.burstCounter--;  // not practical to try to gate this from loop() execution, so we accept the off by one issue and check again
      if (hasRemainingBursts()) {
        pusherOn = true;  // reset the firing cycle
        resetCycle();
      }
    }
  }
  globalState.noidState = pusherOn;
  return pusherOn;
}

uint16_t getRevLogic() {  // Return int with the rev speed.
  if (getDigitalPin(PINREV) || getDigitalPin(PINTRIG)) {
    globalState.currentRevSpeed =
      getDigitalPin(PINMENU) ? (((globalState.targetVelocity - WHEELMINSPEED) * getCurrentFiringProfile().fracVelMultiplier) + WHEELMINSPEED) : globalState.targetVelocity;
  } else {
    if (getDigitalPin(PINMENU) || !ENABLEDECAY) globalState.currentRevSpeed = WHEELMINSPEED;
    else globalState.currentRevSpeed = ((globalState.currentRevSpeed - WHEELMINSPEED) * globalParams.decayMultiplier) + WHEELMINSPEED;
  }
  return globalState.currentRevSpeed;
}

bool isFullAuto() {
  return getCurrentFiringProfile().firingMode == 255;
}
bool hasRemainingBursts() {
  return globalState.burstCounter > 0;
}
void resetBurstCounter() {
  globalState.burstCounter = getCurrentFiringProfile().firingMode;
}

const char* getBootModeLogString() {
  switch (globalState.useMode) {
    case UseBootMode::BOOTCONFIG:
      return "Boot to config.";
      break;
    default:
      return "Boot normally.";
      break;
  }
}

void updateFiringProfileIndex() {
  globalState.currentFiringProfileIndex = getSelectorIndex();
}

void checkScreenUpdate(bool isProfileChanged, uint16_t revLogic) {
}

bool execInterval(unsigned long interval = 500) {
  static unsigned long lastToggle = 0;
  unsigned long now = millis();
  if (now - lastToggle >= interval) {
    lastToggle = now;
    return true;
  } else return false;
}

void bootStandardLoop() {
  updateFiringProfileIndex();  // update current profile index based on slide switch posiioning.
  bool isProfileChanged = globalState.currentFiringProfileIndex != globalState.previousFiringProfileIndex;
  if (isProfileChanged) resetBurstCounter();
  uint16_t revLogic = getRevLogic();  // updates rev logic, the current motor speed.
  //Serial.println(digitalRead(PINREV));
  //Serial.println(digitalRead(PINTRIG));
  //Serial.println(analogRead(PINVOLTAGE));
  //Serial.println(revLogic);
  //esc.writeMicroseconds(revLogic);
  //Serial.println(getSelectorIndex());
  setDigitalPin(PINPUSHER, getCyclingLogic());  // updates cycling logic, state of the pusher.
  checkScreenUpdate(isProfileChanged, revLogic);
  globalState.previousFiringProfileIndex = globalState.currentFiringProfileIndex;
}

void bootConfigLoop() {
  bool isMenuPressed = getDigitalPin(PINMENU);
  if (getDigitalPin(PINMENU)) {
    if (debounceableMenu.isDebounced()) {
      haptic.start();
    }
  } else {
    debounceableMenu.resetDebounce();
    if (haptic.isDone()) {
      haptic.reset();
    }
  }
  globalState.previousIsMenuPressed = isMenuPressed;
  setDigitalPin(PINPUSHER, haptic.getUpdatedStatus());
}

// 5. setup() and loop()
void setup() {
  screenMgr.initDisplay();
  //screenMgr.cycleSprites();
  //setText("Hello World!", 3, 2);
  //screenMgr.updateScreen();

  assignPins();
  updateNoidOffTime();
  initBootVelocity();

  globalState.useMode = getBootMode();  // WIP boot mode implementation

  initESC();  // bind and arm the ESC's for PWM mode

  //digitalWrite(LED_BUILTIN, LOW); //disable built-in LED
  updateFiringProfileIndex();
  UI.updateStatus();
  //Serial.begin(9600);
  //Serial.println("PM Started");
  //Serial.println(getBootModeLogString());
}

void loop() {
  if (globalState.useMode != UseBootMode::BOOTCONFIG) {
    bootStandardLoop();
  } else {
    bootConfigLoop();
  }
  if (execInterval()) UI.updateStatus();
}