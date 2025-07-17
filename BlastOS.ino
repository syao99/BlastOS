/*

BlastOS v0.1 for brushless+solenoid flywheel blasters, based off TagBot Revision 000
By m0useCat

Table of Contents (use ctrl-f):
1. Pinout
2. Types, Core Structs, Required Helpers
3. Global State & Defaults
4. Other Helpers
5. Core Methods
6. setup() and loop()

*/
//#define CTWI_USING_BLOCKING_ACCESS

#include <Servo.h>  // PWM output to ESC's
#include <nI2C.h>   // Screen
//#include <type_traits>
//#include <avr/pgmspace.h>
#include <EEPROM.h>  // Save/load profiles
/*
todo:
implement config UI
*/
#define DEBUGMODE true
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
#define PINPUSHER 10  //10 USE PIN 10 FOR IMPLEMENTATION, 13 IS FOR LED DEBUGGING. PIN 13 WILL ACTUATE THE NOID ON POWERUP, POSSIBLY CAUSING JAMS.
//#define PINPUSHER 10
// Other Params
#define WHEELMINSPEED 1000
#define WHEELMAXSPEED 2000
#define HWMAXVOLTAGE 1680

#define SCREEN_ADDR 0x3C
#define ROW_LAST 7
#define COL_LAST 15
#define ROW_COUNT 8
#define COL_COUNT 16
#define REF_TIME_MS -10000

//#define EEPROMOFFSET 24  // max 1000, add 24 if EEPROM issues occur. 24 may actually need to be a different value depending on save config data
const char* versionText = "BlastOS v0.1";

// 2. Types, Core Structs, Required Helpers
enum class UseBootMode : uint8_t { BOOTMID,
                                   BOOTFRONT,
                                   BOOTBACK,
                                   BOOTCONFIG,
};

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
struct StaticParams {  // not exposed to config yet
  uint16_t minPerCell = 360;
  uint16_t maxPerCell = 420;
  bool enableVoltageSafetyLockout = false;
};
struct GlobalParams {
  uint8_t maxDPS = 20;
  uint8_t noidOnTime = 35;
  uint8_t compLockProfile = 0;  // 0 disabled, 1/2/3 for corresponding profiles
  uint8_t cellCount = 4;
  uint16_t decayConstant = 600;
  uint16_t delayBeforeEndRev = 250;
  GlobalParams() {}
};
struct ProfileParams {
  uint8_t noidDPS;
  uint8_t fracVelPercentage;
  uint8_t firingMode;  // 0: safe, 1: semi, 2-254: burst, 255: auto
  //bool leftyMode = false;
  ProfileParams(
    uint8_t DPS = 20,
    uint8_t fvMultiplier = 50,
    uint8_t mode = 3  //,
    //bool left = false
    )
    : noidDPS(DPS),
      fracVelPercentage(fvMultiplier),
      firingMode(mode) {}
};
struct GlobalState {
  uint16_t targetVelocity = WHEELMINSPEED;
  unsigned long cycleStartTime = 0;
  bool isCycleActive = false;
  bool previousIsFiring = false;
  uint8_t burstCounter = 0;
  bool noidState = false;
  uint16_t noidOffTime;
  uint8_t currentFiringProfileIndex = 0;
  uint8_t previousFiringProfileIndex = 0;
  UseBootMode useMode;
  char* bootModeText;
  GlobalState() {}
  bool isStealthModeEnabled = false;
  uint16_t minVoltage;
  uint16_t maxVoltage;
  bool isUnsafeVoltage = true;
  bool isBootConfigLockout = false;
  float calcdFracVelMultiplier[3];
  float calcdDecayConstant;
  bool enableDecay = true;
};
struct Haptic {
  unsigned long startTime = 0;
  bool isActive = false;  // external control to enable or shut off the cycle. takes effect immediately but can fix that later.
  bool state = false;
  const uint8_t cycleCount;
  const uint16_t onTime;
  const uint16_t offTime;
  uint16_t cycleTime;
  uint16_t duration;
  unsigned long elapsed;
  Haptic(uint8_t count = 2, uint16_t on = 6, uint16_t off = 6)
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
    if (isDone()) return false;
    state = (elapsed % cycleTime) < onTime;
    return state;
    // verify logic in js browser console: for (i = 0; i < 100; ++i) console.log((i % 10) < 4);
  }
  void reset() {
    isActive = false;
  }
  bool allowAction() {
    return (!isActive);
  }
};
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

static const uint8_t spriteSheet[128][8] PROGMEM = {
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
  [49] = { 0x1E, 0x3E, 0x20, 0x3E, 0x3E, 0x20, 0x3E, 0x3E },   // 'm'
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
  [95] = { 0x10, 0x30, 0x70, 0xF0, 0xF0, 0x70, 0x30, 0x10 },   // Triangle Up
  [96] = { 0x18, 0x3C, 0x7E, 0xFF, 0x00, 0x00, 0x00, 0x00 },   // Triangle Right
  [97] = { 0x08, 0x0C, 0x0E, 0x0F, 0x0F, 0x0E, 0x0C, 0x08 },   // Triangle Down
  [98] = { 0x00, 0x00, 0x00, 0x00, 0xFF, 0x7E, 0x3C, 0x18 },   // Triangle Left
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

  bool isScreenCleared;

  bool setTileMapAt(uint8_t val, uint8_t row, uint8_t col) {
    if (row > ROW_LAST) return false;
    if (col > COL_LAST) return false;
    if (tileMap[row][col] == val) return true;
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
    for (uint8_t i = 0; i < sizeof(initCmds); ++i) {
      sendcol(0x00, &initCmds[i], 1);
    }
    screenClear();
  }

  void screenClear() {
    // reserve 1 byte for SSD1306 control, 1 for nI2C register address
    //const uint8_t capacity = CTWI::SIZE_BUFFER - 2;
    for (uint8_t row = 0; row < 8; ++row) {
      uint8_t cmd;
      cmd = 0xB0 | row;  // set row address
      sendcol(0x00, &cmd, 1);
      cmd = 0x00;  // set column low nibble = 0
      sendcol(0x00, &cmd, 1);
      cmd = 0x10;  // set column high nibble = 0
      sendcol(0x00, &cmd, 1);
      // stream each tile’s 8 bytes directly
      for (uint8_t col = 0; col < 16; ++col) {
        for (uint8_t col = 0; col < 8; ++col) {
          uint8_t b = 0;
          sendcol(0x40, &b, 1);
        }
      }
    }
    isScreenCleared = true;
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
    isScreenCleared = false;
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
  void setText(const char* text, uint8_t row, uint8_t startCol = 0) {
    for (uint8_t i = 0; text[i] && startCol + i < COL_COUNT; ++i) {
      if (!setTileMapAt(charToTileID(text[i]), row, startCol + i)) return;
    }
  }
  void setSprite(uint8_t sprite, uint8_t row, uint8_t col) {
    setTileMapAt(sprite, row, col);
  }
  void invertSpan(uint8_t row, uint8_t col = 0, uint8_t span = COL_COUNT) {
    // clamp span so col+span ≤ 16
    if (col > COL_LAST) col = COL_LAST;
    if (row > ROW_LAST) row = ROW_LAST;
    uint8_t maxSpan = (col + span > COL_COUNT ? COL_COUNT - col : span);
    for (uint8_t i = 0; i < maxSpan; ++i) {
      invertTileMapAt(row, col + i);
    }
  }
  void cycleSprites(bool update = true) {
    //Serial.println("test cyclespr");
    uint8_t idx = 0;
    for (uint8_t row = 0; row < ROW_COUNT; ++row) {
      for (uint8_t col = 0; col < COL_COUNT; ++col) {
        if (row > ROW_LAST) break;
        if (col > COL_LAST) break;
        setTileMapAt(idx++, row, col);
        //tileMap[row][col] = idx++;
      }
    }
    if (update) updateScreen();
  }
  void setCheckers(bool update = true) {
    for (uint8_t row = 0; row < ROW_COUNT; ++row) {
      for (uint8_t col = 0; col < COL_COUNT; ++col) {
        tileMap[row][col] = ((row + col) & 1) ? 0x80 : 0x00;
      }
    }
    if (update) updateScreen();
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

char* numToTextPrependAppend(uint16_t num,
                             char prepend = '_',
                             char append = ' ') {
  static char buf[8];
  uint8_t pos = sizeof(buf);
  buf[--pos] = '\0';    // terminate
  buf[--pos] = append;  // set append char

  // fill digits from right to left
  do {
    buf[--pos] = '0' + (num % 10);
    num /= 10;
  } while (num);

  buf[--pos] = prepend;  // set prepend char
  return &buf[pos];
}
char* numToText(uint16_t num) {
  static char buf[6];  // max "65535"+'\0'
  uint8_t pos = sizeof(buf) - 1;
  buf[pos] = '\0';
  do {
    buf[--pos] = '0' + (num % 10);
    num /= 10;
  } while (num);
  return &buf[pos];
}
char* numToText(uint16_t num, uint8_t minWidth) {
  static char buf[6];  // max "65535"+'\0'
  const uint8_t MAX_DIGITS = 5;
  if (minWidth > MAX_DIGITS) minWidth = MAX_DIGITS;
  buf[MAX_DIGITS] = '\0';
  uint8_t pos = MAX_DIGITS;
  uint8_t count = 0;
  do {
    buf[--pos] = '0' + (num % 10);
    num /= 10;
    count++;
  } while (num);
  while (count < minWidth) {
    buf[--pos] = '0';
    count++;
  }
  return &buf[pos];
}
char* voltageToText(uint16_t voltage,
                    uint8_t decimalPlacement = 2,
                    uint8_t minWidth = 2) {
  if (voltage > 9999) voltage = 9999;
  static char buf[8];  // enough for "65535.00\0"
  const uint8_t MAX_IDX = sizeof(buf) - 1;
  buf[MAX_IDX] = '\0';
  uint8_t pos = MAX_IDX;
  // 1) fractional digits
  for (uint8_t i = 0; i < decimalPlacement; ++i) {
    buf[--pos] = '0' + (voltage % 10);
    voltage /= 10;
  }
  // 2) decimal point
  buf[--pos] = '.';
  // 3) integer digits
  uint8_t count = 0;
  do {
    buf[--pos] = '0' + (voltage % 10);
    voltage /= 10;
    ++count;
  } while (voltage);
  // 4) pad with leading spaces up to minWidth
  while (count < minWidth) {
    buf[--pos] = ' ';
    ++count;
  }
  return &buf[pos];
}
char* decayConstantText(uint16_t num, uint8_t minWidth) {
  if (num == UINT16_MAX) return "Off ";
  else return numToText(num, minWidth);
}
struct StatusUIMgr {
  ScreenMgr& scrMgr;
  GlobalParams& globalParams;
  ProfileParams* profileParams;
  GlobalState& globalState;
  StatusUIMgr(ScreenMgr& mgr, GlobalParams& gParams, ProfileParams* pParams, GlobalState& gState)
    : scrMgr(mgr), globalParams(gParams), profileParams(pParams), globalState(gState) {}
  void initStatus(bool updateScr = false) {
    scrMgr.setText(versionText, 0, 2);
    scrMgr.setText(globalState.bootModeText, 1, 0);
    scrMgr.setSprite(102, 1, 11);
    scrMgr.setText(numToText(globalState.targetVelocity), 1, 12);
    scrMgr.setSprite(106, 3, 0);
    scrMgr.setSprite(103, 3, 10);
    scrMgr.setText("DPS", 3, 13);
    scrMgr.setText("v", 7, 11);
    if (updateScr) scrMgr.updateScreen();
  }
  void updateStatus(bool updateScr = true) {
    uint8_t mode = getCurrentFiringProfile().firingMode;
    scrMgr.setText(getModeText(mode), 3, 1);
    scrMgr.setText(getDPSText(), 3, 11);
    scrMgr.setText("S MAX", 5, 6);
    scrMgr.setText(numToText(globalParams.cellCount, 1), 5, 5);
    scrMgr.setSprite(101, 7, 5);
    uint16_t voltage = map(analogRead(PINVOLTAGE), 0, 1023, 0, HWMAXVOLTAGE);
    scrMgr.setText(voltageToText(voltage), 7, 6);
    if (voltage < globalState.minVoltage) {
      globalState.isUnsafeVoltage = true;
      scrMgr.setText("!!LOW BATTERY!!", 6, 0);
    } else if (voltage > globalState.maxVoltage) {
      globalState.isUnsafeVoltage = true;
      scrMgr.setText("!!HIGH BATTERY!!", 6, 0);
    } else {
      globalState.isUnsafeVoltage = false;
      scrMgr.setText("                ", 6, 0);
    }
    if (updateScr) scrMgr.updateScreen();
  }
};


/*enum class MenuItemBehavior : uint8_t { TEXTONLY, SUBPAGE,
                                EDITPROPERTY,
                                ACTION };
struct MenuItemP {
  PGM_P label;                   // flash pointer to null-term string
  MenuItemBehavior behaviors[3];
  uint8_t minVal, maxVal, step;  // for edit property
  union {
    uint16_t pageIndex;  // for sub page
    void (*actionFn)();  // for action
    uint16_t* editPtr;   // for edit property (pointer into RAM)
  } param;
};
static const MenuItemP menuItems[] PROGMEM = {
  { F("Settings"), ItemType::SubMenu, { 0, 0, 0 }, .param.pageIndex = 1 },
  { F("Brightness"), ItemType::EditValue, { 0, 255, 1 }, .param.editPtr = &globalParams.brightness },
  { F("Reset"), ItemType::Action, { 0, 0, 0 }, .param.actionFn = rebootDevice },
  // …
};*/

#define CONFIG_PAGE_COUNT 8
static const char configMenuTexts[CONFIG_PAGE_COUNT][ROW_COUNT][COL_COUNT + 1] PROGMEM = {
  {
    //0
    "BlastOS MainMenu",
    " Global Settings",
    " Velocities     ",
    " Modes          ",
    " About          ",
    " Save           ",
    "Slider & Menu to",
    "Nav & Select    ",
  },
  {
    //1
    "Global Settings ",
    " Go Back        ",
    " Max DPS: ??    ",
    " Noid Time: ??ms",
    " CellCount: ?   ",
    " Coasting: ???  ",  //  < constant for coasting speed computation
    " AfterRev:    ms",  //
    " Comp Lock: ??? ",  // P?: ???? // "RevOffTimeS: ???", // < max 1s",
  },
  {
    //2
    "   Velocities   ",
    " Go Back        ",
    " Low:    ????   ",
    " Medium: ????   ",
    " High:   ????   ",
    "                ",
    "                ",
    "                ",
  },
  {
    //3
    "     Modes      ",
    " Go Back        ",
    " Edit Profile 1 ",
    " Edit Profile 2 ",
    " Edit Profile 3 ",
    "                ",
    "                ",
    "                ",
  },
  {
    //4
    " Edit Profile 1 ",
    " Go Back        ",
    " DPS: 12        ",
    " Fract Vel: 50% ",
    " Mode: Burst3   ",
    "                ",  // <" Lefty Mode: Off",
    "                ",
    "                ",
  },
  {
    //5
    "     About      ",
    "BlastOS Firmware",
    "  by m0useCat   ",
    "   github.com   ",
    "   /syao99      ",
    "   /BlastOS     ",
    " Go Back        ",
    " Factory Reset  ",
  },
  {
    //6
    " Factory Reset  ",
    "This will take a",
    "moment, wipe all",
    "user settings, &",
    "can't be undone.",
    "   Continue?    ",
    " No             ",
    " Yes            ",
  },
  {
    //7
    "                ",
    "                ",
    "  Resetting...  ",
    "  Please Wait   ",
    "                ",
    "                ",
    "                ",
    "                ",
  },
};
char* getConfigMenuText(uint8_t page, uint8_t row) {
  static char buf[COL_COUNT + 1];
  strcpy_P(buf, configMenuTexts[page][row]);
  return buf;
}
const uint8_t configMenuBounds[CONFIG_PAGE_COUNT][2] = {
  { 1, 5 }, { 1, 7 }, { 1, 4 }, { 1, 4 }, { 1, 4 }, { 6, 7 }, { 6, 7 }
};
const uint8_t* getConfigMenuBounds(uint8_t page) {
  return configMenuBounds[page];
}
uint8_t simpleWrap(uint8_t val, int8_t direction, uint8_t rangeMin, uint8_t rangeMax, bool allowMaxWrap = false) {
  if (val == UINT8_MAX) return direction > 0 ? rangeMin : rangeMax;
  if (direction > 0) {
    if (val >= rangeMax) return allowMaxWrap ? UINT8_MAX : rangeMin;
    else return constrain(val + direction, rangeMin, rangeMax);
  } else if (direction < 0) {
    if (val <= rangeMin) return allowMaxWrap ? UINT8_MAX : rangeMax;
    else return constrain(val + direction, rangeMin, rangeMax);
  }
  return val;
}
uint16_t simpleWrap(uint16_t val, int8_t direction, uint16_t rangeMin, uint16_t rangeMax, bool allowMaxWrap = false) {
  if (val == UINT16_MAX) return direction > 0 ? rangeMin : rangeMax;
  if (direction > 0) {
    if (val >= rangeMax) return allowMaxWrap ? UINT16_MAX : rangeMin;
    else return constrain(val + direction, rangeMin, rangeMax);
  } else if (direction < 0) {
    if (val <= rangeMin) return allowMaxWrap ? UINT16_MAX : rangeMax;
    else return constrain(val + direction, rangeMin, rangeMax);
  }
  return val;
}

struct ConfigUIMgr {
  ScreenMgr& scrMgr;
  GlobalParams& globalParams;
  ProfileParams* profileParams;
  GlobalState& globalState;
  const uint16_t* bootVelocities;
  uint8_t currentPage = 0;
  uint8_t cursorIdx = 1;
  void* currentPropertyEdit = nullptr;
  uint8_t currentProfileEdit = 255;
  uint8_t currentBootVelEdit = 255;
  ConfigUIMgr(ScreenMgr& mgr, GlobalParams& gParams, ProfileParams* pParams, GlobalState& gState, uint16_t* bVel)
    : scrMgr(mgr), globalParams(gParams), profileParams(pParams), globalState(gState), bootVelocities(bVel) {}
  void drawConfigPageBase(uint8_t page) {
    for (uint8_t i = 0; i < ROW_COUNT; ++i) {
      scrMgr.setText(getConfigMenuText(currentPage, i), i, 0);
    }
  }
  void drawConfigPageDetails(uint8_t page) {
    switch (page) {
      case 0: break;
      case 1:
        scrMgr.setText(numToText(globalParams.maxDPS, 2), 2, 10);
        scrMgr.setText(numToText(globalParams.noidOnTime, 2), 3, 12);
        scrMgr.setText(numToText(globalParams.cellCount, 1), 4, 12);
        scrMgr.setText(decayConstantText(globalParams.decayConstant, 3), 5, 11);
        scrMgr.setText(numToText(globalParams.delayBeforeEndRev, 3), 6, 11);
        scrMgr.setText(getCompLockProfileText(globalParams.compLockProfile), 7, 12);
        break;
      case 2:
        scrMgr.setText(numToText(bootVelocities[1]), 2, 9);
        scrMgr.setText(numToText(bootVelocities[0]), 3, 9);
        scrMgr.setText(numToText(bootVelocities[2]), 4, 9);
        break;
      case 3:  // profile selection page, nothing to do here since their names are hardcoded.
        break;
      case 4:  // edit profile page
        scrMgr.setText(numToText(currentProfileEdit + 1), 0, 14);
        scrMgr.setText(numToText(profileParams[currentProfileEdit].noidDPS, 2), 2, 6);
        scrMgr.setText(numToText(profileParams[currentProfileEdit].fracVelPercentage), 3, 12);
        scrMgr.setText(getModeText(profileParams[currentProfileEdit].firingMode), 4, 7);
        break;
      case 5:  // about
        break;
      case 6:  // factory reset
        break;
    }
  }
  void drawConfigPage(uint8_t page) {
    drawConfigPageBase(page);
    drawConfigPageDetails(page);
  }
  void updateConfig(uint8_t cursorDirection = 255, bool updateScr = true) {
    if (cursorDirection == 0) {
      if (!currentPropertyEdit) {
        configMenuAction(currentPage, cursorIdx);  //perform menu item action
      } else {
        setPropertyEdit(nullptr);
        if (currentPage == 2) {
          globalState.targetVelocity = WHEELMINSPEED;
          scrMgr.setText("                ", 7);
          globalState.isBootConfigLockout = true;
        }
      };
    }
    if (currentPropertyEdit) {
      if (cursorDirection == 0) {
        scrMgr.invertSpan(cursorIdx);
      }
      if (cursorDirection == 1 || cursorDirection == 2) {
        bool useDir = cursorDirection == 1;
        editCurrentProperty(useDir, currentPage, cursorIdx);
        drawConfigPageDetails(currentPage);
      }
      if (updateScr) scrMgr.updateScreen();
      return;
    }
    drawConfigPage(currentPage);
    if (cursorDirection == 255) {
      scrMgr.setText(">", cursorIdx, 0);
      if (updateScr) scrMgr.updateScreen();
      return;
    }
    updateCursor(cursorDirection);
    if (updateScr) scrMgr.updateScreen();
  }
  void updateCursor(uint8_t direction) {
    if (direction == 0) {
      scrMgr.setText(">", cursorIdx, 0);  // draw cursor w/o moving it.
      return;
    }
    scrMgr.setText(" ", cursorIdx, 0);
    cursorIdx = getUpdatedCursorIdx(direction);
    scrMgr.setText(">", cursorIdx, 0);
  }
  uint8_t getUpdatedCursorIdx(uint8_t direction) {
    const uint8_t* bounds = getConfigMenuBounds(currentPage);
    if (direction == 2) return simpleWrap(cursorIdx, 1, bounds[0], bounds[1]);
    else if (direction == 1) return simpleWrap(cursorIdx, -1, bounds[0], bounds[1]);
    else return 255;
  }
  void setPage(uint8_t newPage, uint8_t newCursorIdx = 255) {
    if (newCursorIdx != 255) cursorIdx = constrain(newCursorIdx, getConfigMenuBounds(newPage)[0], getConfigMenuBounds(newPage)[1]);
    else cursorIdx = getConfigMenuBounds(newPage)[0];
    currentPage = newPage;
  }
  void configMenuAction(uint8_t page, uint8_t action) {
    switch (page) {
      case 0:
        if (action >= 1 && action <= 3) setPage(action);
        if (action == 2) updateConfigFiringProfile();
        switch (action) {
          case 4: setPage(5); return;
          case 5:
            scrMgr.invertSpan(cursorIdx);
            scrMgr.updateScreen();
            save();
            delay(500);
            scrMgr.invertSpan(cursorIdx);
            scrMgr.updateScreen();
            //reboot();
            //cursorIdx = getConfigMenuBounds(0)[0];
            return;
        }
        return;
      case 1:  // global
        switch (action) {
          case 1: setPage(0, 1); return;
          case 2: setPropertyEdit(&globalParams.maxDPS); return;
          case 3: setPropertyEdit(&globalParams.noidOnTime); return;
          case 4: setPropertyEdit(&globalParams.cellCount); return;
          case 5: setPropertyEdit(&globalParams.decayConstant); return;
          case 6: setPropertyEdit(&globalParams.delayBeforeEndRev); return;
          case 7: setPropertyEdit(&globalParams.compLockProfile); return;
        }
        return;
      case 2:  // velocities
        switch (action) {
          case 1: setPage(0, 2); return;
          case 2: currentBootVelEdit = 1; break;
          case 3: currentBootVelEdit = 0; break;
          case 4: currentBootVelEdit = 2; break;
        }
        if (action >= 2 && action <= 4) {
          updateConfigFiringProfile();
          setPropertyEdit(&bootVelocities[currentBootVelEdit]);
          globalState.targetVelocity = bootVelocities[currentBootVelEdit];
          scrMgr.setText("Trigger Enabled ", 7);
          updateNoidOffTime();
          globalState.isBootConfigLockout = false;
        }
        return;
      case 3:  // modes
        switch (action) {
          case 1: setPage(0, 3); return;
          case 2:
            currentProfileEdit = 0;
            setPage(4);
            return;
          case 3:
            currentProfileEdit = 1;
            setPage(4);
            return;
          case 4:
            currentProfileEdit = 2;
            setPage(4);
            return;
        }
        return;
      case 4:  // edit profile
        switch (action) {
          case 1: setPage(3, currentProfileEdit + 2); return;
          case 2: setPropertyEdit(&profileParams[currentProfileEdit].noidDPS); return;
          case 3: setPropertyEdit(&profileParams[currentProfileEdit].fracVelPercentage); return;
          case 4: setPropertyEdit(&profileParams[currentProfileEdit].firingMode); return;
        }
        return;
      case 5:  // about
        switch (action) {
          case 6: setPage(0, 4); return;
          case 7: setPage(6); return;
        }
        return;
      case 6:  // factory reset
        switch (action) {
          case 6: setPage(0, 4); return;
          case 7:
            scrMgr.invertSpan(cursorIdx);
            scrMgr.updateScreen();
            delay(500);
            factoryReset();
            setPage(0);
            scrMgr.updateScreen();
            return;
        }
        return;
    }
  }
  void setPropertyEdit(void* newProperty = nullptr) {
    currentPropertyEdit = newProperty;
  }
  void editCurrentProperty(bool direction, uint8_t page, uint8_t cursorIdx) {  // <-here
    //currentPropertyEdit
    int8_t dirMultiplier = direction ? 1 : -1;
    //Serial.println("EDIT PROPERTY");
    switch (page) {
      case 1:  // global
        switch (cursorIdx) {
          case 2:
            {
              uint8_t* useVal = static_cast<uint8_t*>(currentPropertyEdit);
              *useVal = simpleWrap(*useVal, dirMultiplier, 1, 50);  //val, dir, min, max
              break;
            }
          case 3:
            {
              uint8_t* useVal = static_cast<uint8_t*>(currentPropertyEdit);
              *useVal = simpleWrap(*useVal, dirMultiplier, 1, 50);  //val, dir, min, max
              break;
            }
          case 4:
            {
              uint8_t* useVal = static_cast<uint8_t*>(currentPropertyEdit);
              *useVal = simpleWrap(*useVal, dirMultiplier, 1, 4);  //val, dir, min, max
              break;
            }
          case 5:
            {
              uint16_t* useVal = static_cast<uint16_t*>(currentPropertyEdit);      //decayConstant
              *useVal = simpleWrap(*useVal, dirMultiplier * 25, 200, 1000, true);  //val, dir, min, max;
              break;
            }
          case 6:
            {
              uint16_t* useVal = static_cast<uint16_t*>(currentPropertyEdit);
              *useVal = simpleWrap(*useVal, dirMultiplier * 25, 0, 500);  //val, dir, min, max
              break;
            }
          case 7:
            {
              uint8_t* useVal = static_cast<uint8_t*>(currentPropertyEdit);
              *useVal = simpleWrap(*useVal, dirMultiplier, 0, 3);  //val, dir, min, max
              break;
            }
        }
        break;
      case 2:  // velocities
        {
          if (cursorIdx >= 2 && cursorIdx <= 4) {
            uint16_t* useVal = static_cast<uint16_t*>(currentPropertyEdit);
            *useVal = simpleWrap /*16*/ (*useVal, dirMultiplier * 25, WHEELMINSPEED, WHEELMAXSPEED);  //val, dir, min, max
            globalState.targetVelocity = bootVelocities[currentBootVelEdit];
          }
          break;
        }
      case 3:  // modes - nothing here
        break;
      case 4:  // edit profile
        //currentProfileEdit
        uint8_t multiplier;
        uint8_t min;
        uint8_t max;
        bool allowMaxWrap = false;
        switch (cursorIdx) {
          case 2:  // dps
            multiplier = 1;
            min = 1;
            max = globalParams.maxDPS;
            break;
          case 3:  // fracvel %
            multiplier = 5;
            min = 10;
            max = 90;
            break;
          case 4:  // mode
            multiplier = 1;
            min = 0;
            max = 9;
            allowMaxWrap = true;
            break;
        }
        uint8_t* useVal = static_cast<uint8_t*>(currentPropertyEdit);
        *useVal = simpleWrap(*useVal, dirMultiplier * multiplier, min, max, allowMaxWrap);  //val, dir, min, max
        break;
      case 5:  // about - nothing here
        break;
      case 6:  // factory reset - nothing here
        break;
    }
  }
  void factoryReset() {
    Serial.println("factory reset");
    for (int i = 0; i < EEPROM.length(); i++) {
      if (EEPROM.read(i) != 255) EEPROM.write(i, 255);
    }
    resetUserParamsToDefault();
  }
};

// 3. Global State & Defaults
Servo esc;

// Global, Profile, and State

StaticParams staticParams;

const GlobalParams defaultGlobalParams;
const ProfileParams defaultFiringProfiles[] = {
  // dps, fvMulti, mode i.e. 0: safe, 1: semi, 2-254: burst, 255: auto
  { ProfileParams(defaultGlobalParams.maxDPS, 50, 3) },
  { ProfileParams(defaultGlobalParams.maxDPS, 50, 1) },
  { ProfileParams(defaultGlobalParams.maxDPS, 50, 255) },
};
const uint16_t defaultBootVelocities[] = { 1600, 1450, 2000 };

GlobalParams globalParams;
ProfileParams firingProfiles[3] = defaultFiringProfiles;
uint16_t bootVelocities[3] = { 1600, 1450, 2000 };

ProfileParams configFiringProfile = ProfileParams(globalParams.maxDPS, 50, 1);

void updateConfigFiringProfile() {
  configFiringProfile.noidDPS = globalParams.maxDPS;
}

GlobalState globalState;
Debounceable debounceableTrigger;
Debounceable debounceableMenu;
Haptic haptic = Haptic();
SingleAction btnAction = SingleAction();
ScreenMgr scrMgr;
StatusUIMgr StatusUI{ scrMgr, globalParams, firingProfiles, globalState };
ConfigUIMgr ConfigUI{ scrMgr, globalParams, firingProfiles, globalState, bootVelocities };

uint8_t getSelectorIndex() {
  if (getDigitalPin(PINSELECT1)) return 1;
  if (getDigitalPin(PINSELECT2)) return 2;
  return 0;
}

char* getBootModeText(uint8_t index, bool useCompLock = false) {
  switch (index) {
    case 0: return "Mid Power";
    case 1: return "Low Power";
    case 2: return "High Power";
  }
}

char* getModeText(uint8_t mode) {
  switch (mode) {
    case 0: return "Safe  ";
    case 1: return "Semi  ";
    case 255: return "Auto  ";
    default:
      static char text[7];  // "Burst" + up to 1 digit + '\0'
      snprintf(text, sizeof text, "Burst%u", mode);
      return text;
  }
}

char* getDPSText() {
  uint8_t dps = getCurrentFiringProfile().noidDPS;
  return numToText(dps, 2);
}

char* getCompLockProfileText(uint8_t profile) {
  switch (profile) {
    case 0: return "Off";
    case 1: return "Mid";
    case 2: return "Low";
    case 3: return "Hi ";
    default:
      {
        return numToTextPrependAppend(profile, 'P');
      }
  }
}

ProfileParams getCurrentFiringProfile() {  //<-here
  if (globalState.useMode == UseBootMode::BOOTCONFIG) return configFiringProfile;
  return firingProfiles[globalState.currentFiringProfileIndex];
}

void resetUserParamsToDefault() {
  globalParams = defaultGlobalParams;
  memcpy(firingProfiles, defaultFiringProfiles, sizeof(firingProfiles));
  memcpy(bootVelocities, defaultBootVelocities, sizeof(bootVelocities));
  //Serial.println("reset user params to default");
}

void initBootVelocity() {
  uint8_t bootSelectorIndex = getSelectorIndex();
  bool isCompLockEnabled = globalParams.compLockProfile > 0;
  if (isCompLockEnabled) bootSelectorIndex = globalParams.compLockProfile - 1;
  globalState.targetVelocity = bootVelocities[bootSelectorIndex];
  globalState.bootModeText = getBootModeText(bootSelectorIndex, isCompLockEnabled);
}

void updateNoidOffTime() {
  globalState.noidOffTime = (1000.f / getCurrentFiringProfile().noidDPS) - globalParams.noidOnTime;
}

void initMinMaxVoltage() {
  globalState.minVoltage = staticParams.minPerCell * globalParams.cellCount;
  globalState.maxVoltage = staticParams.maxPerCell * globalParams.cellCount;
}

void initCalcdParam(uint16_t param, float multiplier) {
  return float(param) * multiplier;
}
void initCalcdDecayConstant() {
  globalState.calcdDecayConstant = float(globalParams.decayConstant) * 0.001f;
  globalState.enableDecay = (globalParams.decayConstant == UINT16_MAX);
}
void initFracVelMultiplier() {
  for (uint8_t i = 0; i < 3; i++) {
    globalState.calcdFracVelMultiplier[i] = float(firingProfiles[i].fracVelPercentage) * 0.01f;
  }
}

bool isSafetyLockout() {
  return staticParams.enableVoltageSafetyLockout && globalState.isUnsafeVoltage && !globalState.isBootConfigLockout;
}

// 4. Other Helpers

bool getDigitalPin(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

void setDigitalPin(uint8_t pin, bool isActive) {
  digitalWrite(pin, isActive);
}

// 5. Core Methods

void serialPrintTileMap() {
  for (uint8_t page = 0; page < 8; ++page) {
    for (uint8_t col = 0; col < 16; ++col) {
      char* showVal = numToText(scrMgr.getTileMapAt(page, col), 3);
      Serial.print(showVal);
      Serial.print(' ');
    }
    Serial.println();
  }
  Serial.println();
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

UseBootMode getBootModeIdx() {
  if (getDigitalPin(PINMENU)) return UseBootMode::BOOTCONFIG;
  if (getDigitalPin(PINSELECT1)) return UseBootMode::BOOTFRONT;
  if (getDigitalPin(PINSELECT2)) return UseBootMode::BOOTBACK;
  return UseBootMode::BOOTMID;
  //return UseBootMode::BOOTCONFIG;
}

void initESC() {
  esc.attach(PINESC, WHEELMINSPEED, WHEELMAXSPEED);
  delay(100);
  esc.writeMicroseconds(WHEELMINSPEED);
}

void resetCycle() {
  globalState.cycleStartTime = millis();
}

bool getCyclingLogic() {
  if (isSafetyLockout() || isSafeMode()) return false;
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

uint16_t getHandledRevLogic(bool isMenuPressed, bool revOrTrigIsActive) {  // adds a delay to end of rev
  //return getRevLogic(isMenuPressed, revOrTrigIsActive);
  static unsigned long lastReleaseTime = REF_TIME_MS;
  static bool previousRevOrTrigIsActive = false;
  uint16_t useRevLogic = WHEELMINSPEED;
  if (!revOrTrigIsActive) {
    if (previousRevOrTrigIsActive != revOrTrigIsActive) {
      lastReleaseTime = millis();
      useRevLogic = getRevLogic(isMenuPressed, true);
    }
    if (millis() - lastReleaseTime < globalParams.delayBeforeEndRev) useRevLogic = getRevLogic(isMenuPressed, true);
    else useRevLogic = getRevLogic(isMenuPressed, false);
  } else useRevLogic = getRevLogic(isMenuPressed, true);
  previousRevOrTrigIsActive = revOrTrigIsActive;
  return useRevLogic;
}
uint16_t getFracVelTargetSpeed() {
  return (((globalState.targetVelocity - WHEELMINSPEED) * globalState.calcdFracVelMultiplier[globalState.currentFiringProfileIndex]) + WHEELMINSPEED);
}
uint16_t getRevLogic(bool isMenuPressed, bool revOrTrigIsActive) {  // Return int with the rev speed.
  static unsigned long lastReleaseTime = REF_TIME_MS;
  static bool previousRevOrTrigIsActive = false;
  static uint16_t currentTargetSpeed = WHEELMINSPEED;
  float currentRevSpeed = WHEELMINSPEED;
  if (isSafetyLockout() || isSafeMode()) return WHEELMINSPEED;
  if (revOrTrigIsActive) {
    if (isMenuPressed) currentTargetSpeed = getFracVelTargetSpeed();
    else currentTargetSpeed = globalState.targetVelocity;
    currentRevSpeed = currentTargetSpeed;
  } else {
    if (globalState.isStealthModeEnabled || !globalState.enableDecay) {
      lastReleaseTime = REF_TIME_MS;
      currentRevSpeed = WHEELMINSPEED;
    } else {
      if (previousRevOrTrigIsActive != revOrTrigIsActive && !revOrTrigIsActive) lastReleaseTime = millis();
      float t = (millis() - (lastReleaseTime)) / 1000.f;
      currentRevSpeed = ((currentTargetSpeed - WHEELMINSPEED) * expf(-globalState.calcdDecayConstant * t)) + WHEELMINSPEED;
      if (currentRevSpeed < WHEELMINSPEED + 50) currentRevSpeed = WHEELMINSPEED;
    }
  }
  previousRevOrTrigIsActive = revOrTrigIsActive;
  //Serial.print("current target speed: ");
  //Serial.print(currentTargetSpeed);
  //Serial.println();
  return (uint16_t)(currentRevSpeed + 0.5f);
}
bool isRevOrTrigActive() {
  return getDigitalPin(PINREV) || getDigitalPin(PINTRIG);
}

bool isSafeMode() {
  return getCurrentFiringProfile().firingMode == 0;
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
    default:
      return "Boot normally.";
  }
}

void updateFiringProfileIndex() {
  globalState.currentFiringProfileIndex = getSelectorIndex();
}

bool execInterval(unsigned long interval = 2000) {
  static unsigned long lastToggle = 0;
  unsigned long now = millis();
  if (now - lastToggle >= interval) {
    lastToggle = now;
    return true;
  } else return false;
}

bool shouldScreenUpdate(bool isProfileChanged, bool revOrTrigIsActive, bool isMenuPressed) {
  if (isMenuPressed && !revOrTrigIsActive) return false;
  if (isProfileChanged) return true;
  if (revOrTrigIsActive) return false;
  if (isMenuPressed) return false;
  return execInterval();
}

void initAllParams() {
  updateConfigFiringProfile();
  initMinMaxVoltage();
  initCalcdDecayConstant();
  initFracVelMultiplier();
}

void bootStandardLoop() {
  updateFiringProfileIndex();  // update current profile index based on slide switch posiioning.
  static bool previousIsMenuPressed;
  static bool previousIsStealthModeEnabled;
  static bool stealthLockout = false;
  bool isMenuPressed = getDigitalPin(PINMENU);
  bool revOrTrigIsActive = isRevOrTrigActive();
  bool isProfileChanged = globalState.currentFiringProfileIndex != globalState.previousFiringProfileIndex;
  if (revOrTrigIsActive) stealthLockout = true;
  if (!revOrTrigIsActive && !isMenuPressed) stealthLockout = false;
  globalState.isStealthModeEnabled = !stealthLockout && !revOrTrigIsActive && isMenuPressed;
  if (isProfileChanged) {
    resetBurstCounter();
    updateNoidOffTime();
  }
  uint16_t revLogic = getHandledRevLogic(isMenuPressed, revOrTrigIsActive);  // updates rev logic, the current motor speed.
  bool cyclingLogic = getCyclingLogic();
  esc.writeMicroseconds(revLogic);
  setDigitalPin(PINPUSHER, cyclingLogic);  // updates cycling logic, state of the pusher.
#if DEBUGMODE
  Serial.println(revLogic);
  setDigitalPin(LED_BUILTIN, cyclingLogic);
#endif
  if (globalState.isStealthModeEnabled != previousIsStealthModeEnabled && globalState.isStealthModeEnabled) scrMgr.screenClear();
  if (!globalState.isStealthModeEnabled && shouldScreenUpdate(isProfileChanged, revOrTrigIsActive, isMenuPressed)) StatusUI.updateStatus();
  globalState.previousFiringProfileIndex = globalState.currentFiringProfileIndex;
  previousIsMenuPressed = isMenuPressed;
  previousIsStealthModeEnabled = globalState.isStealthModeEnabled;
}

void bootConfigLoop() {
  //scrMgr.loopInvertAll();
  static bool previousIsMenuPressed = getDigitalPin(PINMENU);
  bool isMenuPressed = getDigitalPin(PINMENU);
  if (isMenuPressed != previousIsMenuPressed) {
    //uint8_t selectorIdx = getSelectorIndex();
    if (isMenuPressed) {
      if (debounceableMenu.isDebounced()) {
        if (haptic.allowAction()) {
          uint8_t selection = getSelectorIndex();
          ConfigUI.updateConfig(selection);
          if (globalState.isBootConfigLockout) haptic.start();
        }
      }
    } else {
      debounceableMenu.resetDebounce();
      if (haptic.isDone()) haptic.reset();
    }
  }
  if (!globalState.isBootConfigLockout) {
    bool cyclingLogic = getCyclingLogic();
    uint16_t revLogic = getHandledRevLogic(false, isRevOrTrigActive());  // updates rev logic, the current motor speed.
    esc.writeMicroseconds(revLogic);
    setDigitalPin(PINPUSHER, cyclingLogic);  // updates cycling logic, state of the pusher.
#if DEBUGMODE
    Serial.println(revLogic);
    setDigitalPin(LED_BUILTIN, cyclingLogic);
#endif
  } else {
    esc.writeMicroseconds(WHEELMINSPEED);
    setDigitalPin(PINPUSHER, haptic.getUpdatedStatus());
#if DEBUGMODE
    Serial.println(WHEELMINSPEED);
#endif
  }
  previousIsMenuPressed = isMenuPressed;
}

static constexpr int EEPROM_BASE_ADDR = 0;
static constexpr uint16_t EEPROM_MAGIC = 0xA5A5;

// compute total sizes
static constexpr size_t SIZE_GLOBAL = sizeof(globalParams);
static constexpr size_t SIZE_PROFILES = sizeof(firingProfiles);
static constexpr size_t SIZE_VELOCITIES = sizeof(bootVelocities);

void load() {
  int addr = EEPROM_BASE_ADDR;
  uint16_t magic;
  EEPROM.get(addr, magic);
  if (magic != EEPROM_MAGIC) {
    // first‐boot or version mismatch: leave defaults or init here
    return;
  }
  addr += sizeof(magic);

  EEPROM.get(addr, globalParams);
  addr += SIZE_GLOBAL;

  EEPROM.get(addr, firingProfiles);
  addr += SIZE_PROFILES;

  EEPROM.get(addr, bootVelocities);
  addr += SIZE_VELOCITIES;

  // optional: verify addr ≤ EEPROM.length()
}
/*
static int updateBlock(int addr, const void* data, size_t len) {
  auto p = (const uint8_t*)data;
  for (size_t i = 0; i < len; ++i) {
    addr = EEPROM.update(addr, p[i]);
  }
  return addr;
}
*/
void save() {
  int addr = EEPROM_BASE_ADDR;

  EEPROM.put(addr, EEPROM_MAGIC);
  addr += sizeof(EEPROM_MAGIC);

  EEPROM.put(addr, globalParams);
  addr += SIZE_GLOBAL;

  EEPROM.put(addr, firingProfiles);
  addr += SIZE_PROFILES;

  EEPROM.put(addr, bootVelocities);
  addr += SIZE_VELOCITIES;
}

void serialPrintDumpEEPROMHex() {
  for (int addr = 0; addr < EEPROM.length(); ++addr) {
    uint8_t b = EEPROM.read(addr);
    Serial.print(addr);
    Serial.print(":0x");
    if (b < 0x10) Serial.print('0');
    Serial.println(b, HEX);
  }
}

void serialPrintDumpEEPROMDec() {
  for (int addr = 0; addr < EEPROM.length(); ++addr) {
    uint8_t b = EEPROM.read(addr);
    Serial.print(addr);
    Serial.print(": ");
    Serial.println(b, DEC);  // print value in decimal (0–255)
  }
}

void setupWrapped(bool firstTime = true) {
  if (firstTime) {
    scrMgr.initDisplay();
    assignPins();
  } else {
    scrMgr.screenClear();
  }
  load();

  initAllParams();
  globalState.useMode = getBootModeIdx();  // WIP boot mode implementation
  updateFiringProfileIndex();
  if (globalState.useMode != UseBootMode::BOOTCONFIG) {
    initBootVelocity();
    updateNoidOffTime();
    globalState.isBootConfigLockout = false;
    StatusUI.initStatus();
    StatusUI.updateStatus();
  } else {
    globalState.isBootConfigLockout = true;
    updateNoidOffTime();
    ConfigUI.updateConfig();
  }

#if DEBUGMODE
  Serial.begin(9600);
  Serial.println("Welcome to BlastOS");
  Serial.println(getBootModeLogString());
#endif
  initESC();
}

void reboot() {
  delay(2000);
  resetUserParamsToDefault();
  setupWrapped(false);
}

// 6. setup() and loop()
void setup() {
  setupWrapped();
}

void loop() {
  if (globalState.useMode != UseBootMode::BOOTCONFIG) {
    bootStandardLoop();
  } else {
    bootConfigLoop();
  }
}