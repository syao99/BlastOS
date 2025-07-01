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

#include <Servo.h> // PWM output to ESC's
#include <nI2C.h> // Screen
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
#define PINPUSHER 13 //10 USE PIN 10 FOR IMPLEMENTATION, 13 IS FOR LED DEBUGGING. PIN 13 WILL ACTUATE THE NOID ON POWERUP, POSSIBLY CAUSING JAMS.
//#define PINPUSHER 10
// Other Params
#define WHEELMINSPEED 1000
#define WHEELMAXSPEED 2000
#define MAXVOLTAGE 16.8
#define ENABLEDECAY true
//#define EEPROMOFFSET 24  // max 1000, add 24 if EEPROM issues occur. 24 may actually need to be a different value depending on save config data

// 2. Types
enum class UseBootMode : uint8_t { BOOTMID, BOOTFRONT, BOOTBACK, BOOTCONFIG }; // zero: default on tb2, middle position phanta.
//maybe refactor this to just uint8_t, zero for config. idk yet.

struct Debounceable {
  const uint8_t debounceTime;
  unsigned long debounceStartTime = 0;
  Debounceable(uint8_t time = 10) : debounceTime(time) {}
  void resetDebounce() { debounceStartTime = millis(); }
  bool isDebounced() { return (millis() - debounceStartTime) >= debounceTime; }
};
struct GlobalParams {
  uint8_t maxDPS = 20;
  uint8_t noidOnTime = 25;
  uint8_t compLockProfile = 0; // 0 disabled, 1/2/3 for corresponding profiles
  uint16_t voltageThreshold = 100;
  float decayMultiplier = 0.99f;
  GlobalParams() {}
};
struct ProfileParams {
  uint8_t noidDPS;
  float fracVelMultiplier;
  uint8_t firingMode; // 0: safe, 1: semi, 2-254: burst, 255: auto
  //bool leftyMode = false;
  ProfileParams(
      uint8_t DPS = 12,
      float fvMultiplier = 0.5f,
      uint8_t mode = 3//,
      //bool left = false
    ) :
    noidDPS(DPS),
    fracVelMultiplier(fvMultiplier),
    firingMode(mode)
  {}
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
  GlobalState() {}
};
struct Haptic { // for haptics
  unsigned long startTime = 0;
  bool isActive = false; // external control to enable or shut off the cycle. takes effect immediately but can fix that later.
  bool state = false;
  const uint8_t cycleCount;
  const uint16_t onTime;
  const uint16_t offTime;
  uint16_t cycleTime;
  uint16_t duration;
  unsigned long elapsed;
  Haptic(uint8_t count = 3, uint16_t on = 30, uint16_t off = 30) :
    cycleCount(count),
    onTime(on),
    offTime(off)
  {}
  void start() {
    if (isActive) return;
    startTime = millis();
    isActive = true;
    cycleTime = onTime + offTime;
    duration = cycleTime * cycleCount;
  }
  bool isDone() { return elapsed > duration; }
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
struct RateLimitedAction { // rate limited action. use requestExec() to call, then if (shouldExec()) function() to call.
  const uint8_t delay;
  unsigned long startTime;
  bool isActive = false;
  bool shouldExec = false;
  RateLimitedAction(uint8_t timing = 100) : delay(timing) {}
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
struct SingleAction {
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
};
/*struct Cycler { // for haptics, but could theoretically also be used for re-implementing firing logic!
  unsigned long startTime = 0;
  bool controlIsActive = false;
  bool hasStarted = false;
  uint8_t cycleCount;
  uint16_t onTime;
  uint16_t offTime;
  Cycler(uint8_t count = 3, uint16_t on = 6, uint16_t off = 6) :
    cycleCount(count),
    onTime(on),
    offTime(off)
  {}
  void start() {
    startTime = millis();
    controlIsActive = true;
    hasStarted = true;
  }
  bool getStatus() {
    if (!hasStarted) return;
    unsigned long elapsed = millis() - startTime;
    uint16_t totalCycle = onTime + offTime;
    bool shouldBeOn = (elapsed % totalCycle) < onTime;
    if (!controlIsActive) {
      if (!shouldBeOn) hasStarted = false;
    }
    // verify logical premise in js browser console:
    //for (i = 0; i<100; i++) {
    //  console.log((i % 10)<4);
    //}
  }
  void stop() {
    controlIsActive = false;
  }
};*/

// 3. Global State & Defaults
Servo esc;

// Global, Profile, and State

GlobalParams globalParams;
ProfileParams firingProfiles[] = { // dps, fvMulti, mode i.e. 0: safe, 1: semi, 2-254: burst, 255: auto
  {ProfileParams(globalParams.maxDPS, 0.5f, 3)},
  {ProfileParams(globalParams.maxDPS, 0.5f, 1)},
  {ProfileParams(globalParams.maxDPS, 0.5f, 255)}
};
uint16_t bootVelocities[] = {1200, 1400, 1600};
GlobalState globalState;
Debounceable debounceableTrigger;
Debounceable debounceableMenu;
Haptic haptic = Haptic();
SingleAction btnAction = SingleAction();

//TileScreen tileScreen;

uint8_t getSelectorIndex() {
  if (getDigitalPin(PINSELECT1)) return 1;
  if (getDigitalPin(PINSELECT2)) return 2;
  return 0;
}

ProfileParams& getCurrentFiringProfile() {
  return firingProfiles[globalState.currentFiringProfileIndex];
}

void initBootVelocity() {
  globalState.targetVelocity = bootVelocities[getSelectorIndex()];
}

void updateNoidOffTime() { globalState.noidOffTime = (1000.f / getCurrentFiringProfile().noidDPS) - globalParams.noidOnTime; }

//uint16_t deprecated_noidOffTime = findNoidOffTime(deprecated_MAINDPS);



// 4. Core Methods


bool getDigitalPin(uint8_t pin) { return digitalRead(pin) == LOW; }

void setDigitalPin(uint8_t pin, bool isActive) { digitalWrite(pin, isActive); }

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
  if (!debounceableTrigger.isDebounced()) return globalState.noidState; // if not debounced, return previous value
  bool pusherOn = false;
  bool isTriggerPulled = getDigitalPin(PINTRIG);
    /*debounceableTrigger.isDebounced() ?
      getDigitalPin(PINTRIG) :
      globalState.previousIsTriggerPulled;
  globalState.previousIsTriggerPulled = isTriggerPulled;*/
  if (!globalState.isCycleActive) { // no active cycle is happening, start a new one
    if (isTriggerPulled) {
      globalState.isCycleActive = true;
      pusherOn = true;
      resetCycle();
      resetBurstCounter();
    }
  } else { // if cycle has already begun, we need to ensure it is on for the standard noid on time
    unsigned long currentCycleTime = millis() - globalState.cycleStartTime;
    if (currentCycleTime < (globalParams.noidOnTime + globalState.noidOffTime)) { // if cycle time is within the entire cycling time frame
      if (currentCycleTime < globalParams.noidOnTime) { // if cycling time is within standard noid on time, keep it on.
        pusherOn = true;
      }
    }
    else if (!isTriggerPulled) { // trigger released, end the cycle asap but allow it to finish to avoid jams.
      globalState.isCycleActive = false;
      //resetBurstCounter();
      debounceableTrigger.resetDebounce();
    }
    else if (isFullAuto()) {
      pusherOn = true; // reset the firing cycle
      resetCycle();
    }
    else if (hasRemainingBursts()) {
      globalState.burstCounter--; // not practical to try to gate this from loop() execution, so we accept the off by one issue and check again
      if (hasRemainingBursts()) {
        pusherOn = true; // reset the firing cycle
        Serial.println(globalState.burstCounter);
        resetCycle();
      }
    }
  }
  globalState.noidState = pusherOn;
  return pusherOn;
}

unsigned short getRevLogic() { // Return int with the rev speed.
  if (getDigitalPin(PINREV) || getDigitalPin(PINTRIG)) {
    globalState.currentRevSpeed =
      getDigitalPin(PINMENU) ?
        globalState.targetVelocity * getCurrentFiringProfile().fracVelMultiplier :
        globalState.targetVelocity;
  }
  else {
    if (getDigitalPin(PINMENU) || !ENABLEDECAY) globalState.currentRevSpeed = WHEELMINSPEED;
    else globalState.currentRevSpeed = ((globalState.currentRevSpeed - WHEELMINSPEED) * globalParams.decayMultiplier) + WHEELMINSPEED;
  }
  return globalState.currentRevSpeed;
}

bool isFullAuto() { return getCurrentFiringProfile().firingMode == 255; }
bool hasRemainingBursts() { return globalState.burstCounter > 0; }
void resetBurstCounter() { globalState.burstCounter = getCurrentFiringProfile().firingMode; }


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

void bootStandardLoop() {
  globalState.currentFiringProfileIndex = getSelectorIndex(); // update current profile index based on slide switch posiioning.
  if (globalState.currentFiringProfileIndex != globalState.previousFiringProfileIndex) resetBurstCounter();
  unsigned short revLogic = getRevLogic(); // updates rev logic, the current motor speed.
  //Serial.println(digitalRead(PINREV));
  //Serial.println(digitalRead(PINTRIG));
  //Serial.println(analogRead(PINVOLTAGE));
  //Serial.println(revLogic);
  //esc.writeMicroseconds(revLogic);
  setDigitalPin(PINPUSHER, getCyclingLogic()); // updates cycling logic, state of the pusher.
  //digitalWrite(PINPUSHER, getCyclingLogic()); // updates cycling logic, state of the pusher.
  globalState.previousFiringProfileIndex = globalState.currentFiringProfileIndex;
}

void bootConfigLoop() {
  bool isMenuPressed = getDigitalPin(PINMENU);
  if (getDigitalPin(PINMENU)) {
    if (debounceableMenu.isDebounced()) {
      haptic.start();
    }
  }
  else {
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
  assignPins();
	updateNoidOffTime();
  initBootVelocity();

  globalState.useMode = getBootMode(); // WIP boot mode implementation

	initESC(); // bind and arm the ESC's for PWM mode
	
	//digitalWrite(LED_BUILTIN, LOW); //disable built-in LED
	Serial.begin(9600);
	Serial.println("PM Started");
  Serial.println(getBootModeLogString());
}

void loop() {
  if (globalState.useMode != UseBootMode::BOOTCONFIG) {
    bootStandardLoop();
  }
  else {
    bootConfigLoop();
  }
}