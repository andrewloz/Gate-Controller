#include <Arduino.h>

const int lCurrent = A0;
const int rCurrent = A1;
const int leftStop = A2;
const int rightStop = A3;

const int openA = D1;
const int openB = D2;
const int blockSense = D3;
const int unused = D4;

const int lMotor = D5;  // Close
const int rMotor = D6;  // Open
const int unlockPin = D7;
const int lockPin = D8;

const unsigned long debounceDelay = 50;
const unsigned long rampTime = 100;
const unsigned long rampDownTime = 50;

const int maxSpeed = 255;
const int startSpeed = 55;
const int rampIncrement = 20;
const int slowSpeed = 87;

// Stall detection
const int slowStall = 500;  // ~ 4096*(600/5000) 600mA
const int fastStall = 2048; // ~ 4096*(2500/5000) 2.5A

// Open/Close fast time
const unsigned long lockTime = 100;    // 100ms
const unsigned long stopTime = 500;  // 500ms
const unsigned long fastTime = 5000;  // 5 seconds
const unsigned long slowPercent = 50; // 50% of fast time

// Pedestrian open fast time target
const unsigned long pedTime = 1500;  // 1.5 seconds

typedef enum {
  UNKNOWN,  // Just started up, no end sensor detecting
  OPEN,     // End sensor detecting open
  CLOSED,   // End sensor detecting closed
  MID       // Stopped in the middle, but location estimate known
} LocState;

typedef enum {
  STOPPED,      // 0
  UNLOCKING,    // 1
  RAMPING_UP,   // 2
  RAMPING_DOWN, // 3
  FULL_SPEED,   // 4
  SLOW,         // 5
  STOPPING,     // 6
  LOCKING,      // 7
} MoveState;

typedef enum {
  NONE,
  OPENING,
  CLOSING,
} DirState;

typedef enum {
  OPEN_T,
  CLOSE_T,
  PEDESTRIAN_T
} Target;

const MoveState nextAction[] = {
  UNLOCKING,  // UNLOCKING
  RAMPING_UP,  // RAMPING_UP
  FULL_SPEED,  // FULL_SPEED
  RAMPING_DOWN,  // RAMPING_DOWN
  SLOW,  // SLOW
  STOPPING,  // STOPPED
  LOCKING,  // LOCKING
  STOPPED
};

// Stop sensor quiescent values.  Should be saved in EEPROM, but for now initial
// values are hard-coded.
int leftStopRestVal = 2048;
int rightStopRestVal = 2048;
int stopTrigThreshold = 150;
// Current values
int leftStopVal = 0;
int rightStopVal = 0;

LocState locState = UNKNOWN;
MoveState moveState = STOPPED;
DirState dirState = NONE;
Target target = OPEN_T;

// Time in fast state updated as gate is moving at fast speed.
// unsigned long  fastTime = 0;

// Current monitor values
int lCurrentVal = 0;
int rCurrentVal = 0;

// Inout event start times, for debounce
unsigned long openAChangeTime = 0;
unsigned long openBChangeTime = 0;
unsigned long blockSenseChangeTime = 0;
unsigned long openAConfTime = 0;
unsigned long openBConfTime = 0;
unsigned long blockSenseConfTime = 0;

unsigned long lastMoveStateTime = 0;
// unsigned long lastRampChangeTime = 0;
unsigned long timeToNextChange = 0;
unsigned long timeAtFullSpeed = 0;

int rampLevel = 0;

bool openAActive = false;
bool openBActive = false;
bool blockSenseActive = false;

// Input states as of last read
// Note that an active state once dealt with, goes to inactive until there is a new activity.
int openAState = LOW;
int openBState = LOW;
int blockSenseState = LOW;

bool openWhenStopped = false;



// put function declarations here:
void logit();
void readInputs();
void startMoving();
void stopMoving();
bool anyStopSensorActive() {
  return abs(leftStopVal - leftStopRestVal) > stopTrigThreshold || abs(rightStopVal - rightStopRestVal) > stopTrigThreshold;
}

bool lateStopActive() {
  int val = dirState == CLOSING ? leftStopVal : rightStopVal;
  int restVal = dirState == CLOSING ? leftStopRestVal : rightStopRestVal;
  return abs(val - restVal) > stopTrigThreshold;
}

bool openActive() {
  static bool prevActive = false;
  bool active = rightStopVal - rightStopRestVal > stopTrigThreshold;
  if (active && !prevActive) {
    Serial.println("Open active");
    prevActive = active;
  }
  return active;
}

bool closedActive() {
  static bool prevActive = false;
  bool active = leftStopVal - leftStopRestVal < -stopTrigThreshold;
  if (active && !prevActive) {
    Serial.println("Closed active");
    prevActive = active;
  }
  return active;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(openA, INPUT_PULLUP);
  pinMode(openB, INPUT_PULLUP);
  pinMode(blockSense, INPUT_PULLUP);
  pinMode(lMotor, OUTPUT);
  pinMode(rMotor, OUTPUT);
  pinMode(unlockPin, OUTPUT);
  pinMode(lockPin, OUTPUT);
}

void loop() {
  // Update all inputs
  readInputs();
  if (closedActive()) {
    // Serial.println("Closed");
    locState = CLOSED;
  } else if (openActive()) {
    // Serial.println("Open");
    locState = OPEN;
  }
  logit();
  // Test current state, action depends on both DirState, possibly LocState and MoveState
  // digins take precedence over analogs
  if (blockSenseActive) {
    blockSenseActive = false;
    if (moveState != STOPPED  && moveState != STOPPING && dirState == CLOSING) {
      stopMoving();
      openWhenStopped = true;
    }
  } else if (openAActive || openBActive) {
    // TODO: Consider ignoring repeat press within say 1-2 secs to allow for dropouts when approaching gate
    if (moveState != STOPPED && moveState != STOPPING) {
      // Maybe ramp down?  See how the gate sounds with this.
      stopMoving();
    } else {
      if (locState == OPEN || locState == MID) {
        // FIXME: A bit dubious, should explicitly use A or B to decide on action, or treat restart as full open
        target = CLOSE_T;
        Serial.print("Open, target is ");
        Serial.println(target);
      } else if (locState == CLOSED) {
        Serial.println("Closed, so open");
        target = (openAActive) ? OPEN_T : PEDESTRIAN_T;
      } else { // UNKNOWN
        Serial.println("Unknown state, defaulting to close");
        target = CLOSE_T;
      }
      rampLevel = 0;
      startMoving();
    }
    openAActive = false;
    openBActive = false;
  } else {
    // If none of the digins are active, and we are moving, do stall detection
    // and time dependent actions.
    if (moveState == STOPPED && openWhenStopped) {
      target = OPEN_T;
      startMoving();
      openWhenStopped = false;
    } else if (moveState != STOPPED) {
      long allowedCurrent = (long)rampLevel * 2048L / 255L + 100L;
      long current = max(lCurrentVal, rCurrentVal);
      if (moveState == FULL_SPEED) {
          timeAtFullSpeed = millis() - lastMoveStateTime;
      }
      if (moveState > UNLOCKING && current > allowedCurrent) {
        stopMoving();
        Serial.println("Stalled");
        // Fast stop, then go to opening if was closing. For opening, just stop.
        if (dirState == CLOSING) {
          openWhenStopped = true;
        }
      } else if (moveState == FULL_SPEED && anyStopSensorActive()) {
        // If we are at full speed and a stop sensor is active, stop.
        Serial.println("Stop sensor active at full speed, stopping");
        stopMoving();
        timeToNextChange = stopTime;
      } else if (moveState == FULL_SPEED && target == PEDESTRIAN_T && millis() - lastMoveStateTime > pedTime) {
        Serial.println("Pedestrian time up, stopping");
        moveState = RAMPING_DOWN;
        timeToNextChange = 1;
      } else if (millis() - lastMoveStateTime > timeToNextChange) {
        Serial.print("Time for next change from ");
        Serial.print(moveState);
        Serial.print(" to ");
        if (moveState == UNLOCKING) {
          Serial.println("End of unlock");
          digitalWrite(unlockPin, LOW);
          // digitalWrite(lockPin, LOW);
        } else if (moveState == LOCKING) {
          Serial.println("End of lock");
          // digitalWrite(unlockPin, LOW);
          digitalWrite(lockPin, LOW);
          moveState = STOPPED;
        }
        if (moveState == UNLOCKING && (locState == CLOSED || locState == OPEN)) {
          timeAtFullSpeed = fastTime;
        }
        // if (moveState == UNLOCKING && !lateStopActive() && (timeAtFullSpeed < 500 || locState == UNKNOWN)) {
        if (moveState == UNLOCKING && !(locState == CLOSED || locState == OPEN) && (timeAtFullSpeed < 500 || locState == UNKNOWN)) {
          moveState = SLOW;
        } else if (moveState == UNLOCKING) {
          moveState = RAMPING_UP;
        }
        Serial.println(moveState);
        lastMoveStateTime = millis();
        switch (moveState) {
          case RAMPING_UP: {
            Serial.println("Ramping up");
            rampLevel = rampLevel == 0 ? startSpeed : rampLevel + rampIncrement;
            if (rampLevel >= 255) {
              rampLevel = 255;
              // moveState = FULL_SPEED; - done by nextAction
              // If we stopped in the middle for some reason, limit the return time to the actual previous time at full speed.
              if (timeAtFullSpeed < fastTime - 400) {
                // valid if always reverse from midway
                timeToNextChange = timeAtFullSpeed;
              } else {
                timeToNextChange = fastTime;
              }
              timeAtFullSpeed = 0;
              moveState = FULL_SPEED;
            } else {
              timeToNextChange = rampTime;
            }
            int pin = dirState == OPENING ? lMotor : rMotor;
            analogWrite(pin, rampLevel);
            if (!anyStopSensorActive()) {
              locState = MID;
            }
          }
            break;
          case RAMPING_DOWN: {
            Serial.println("Ramping down");
            rampLevel -= rampIncrement;
            if (rampLevel <= slowSpeed) {
              rampLevel = slowSpeed;
              timeToNextChange = 100; // #TESTING#
              if (target == PEDESTRIAN_T) {
                moveState = STOPPING;
              } else {
                moveState = SLOW;
              }
            } else {
              timeToNextChange = rampDownTime;
            }
            int dpin = dirState == OPENING ? lMotor : rMotor;
            analogWrite(dpin, rampLevel);
          }
            break;
          case FULL_SPEED:
            Serial.println("Full speed");
            moveState = RAMPING_DOWN;
            timeToNextChange = rampDownTime;
            break;
          case SLOW:
            // Use sensors here to determine if we are at the end.
            Serial.println("Slow");
            if (closedActive()) {
              locState = CLOSED;
              moveState = STOPPING;
            } else if (openActive()) {
              locState = OPEN;
              moveState = STOPPING;
            } else {
              locState = MID;
            }
            if (lateStopActive()) {
              moveState = STOPPING;
              locState = dirState == OPENING ? OPEN : CLOSED;
            } else {
              moveState = SLOW;
            }
            if (timeToNextChange < 100) {
              Serial.println("Time to next change < 100");
              timeToNextChange = 100;
            }
            break;
          case STOPPED:
            Serial.println("Stopped");
            lastMoveStateTime = 0;
            timeToNextChange = 0;
            break;
          case UNLOCKING:
            Serial.println("Unlocking");
            digitalWrite(unlockPin, HIGH);
            timeToNextChange = lockTime;
            break;
          case LOCKING:
            Serial.println("Locking");
            digitalWrite(lockPin, HIGH);
            timeToNextChange = lockTime;
            moveState = STOPPED;
            break;
          case STOPPING:
            stopMoving();
            moveState = LOCKING;
            break;
        }

      }
    } else if (locState == CLOSED) {
      // If stopped and CLOSED, check for involuntary movement
    }
  }

  // else there is no digin active, so if we are moving deal with time dependent and stall detection.
  // Stall check first, if stalled, then if closing, change to opening, if opening, just stop.

  // If stopped (TODO:) check for involuntary movement, and resist it using escalating motor
  // force, ignoring stall current.

  // If moving fast, update the quiescent levels of the stops sensor slowly.
}

// put function definitions here:
void logit() {
  static unsigned logTime = 0;
  if (logTime == 0 || millis() - logTime > 500) {
    logTime = millis();
    char buf[100];
    snprintf(buf, sizeof(buf), "%d %d %d  %d %d %d %d %d %d %ld  %d %d %d  %d %d\n", openAActive, openBActive, blockSenseState,
      leftStopVal, rightStopVal, lCurrentVal, rCurrentVal, rampLevel, moveState, timeToNextChange,
      locState, dirState, target,  openActive(), closedActive());
    Serial.print(buf);
  }
}

void readAnInput(int pin, int &val, unsigned long &changeTime,unsigned long  &confTime, bool &active) {
  // Read an input
  int newVal = digitalRead(pin);
  if (newVal != HIGH && changeTime == 0) {
    changeTime = millis();
  } else if (changeTime - confTime > 1000 && newVal != HIGH && millis() - changeTime > debounceDelay) {
    val = !newVal;
    active = true;
    confTime = changeTime;
  } else if (newVal == HIGH) {
    changeTime = 0;
    val = LOW;
  }
}

void readBlockSense() {
  // Read block sense, we don't mind chatter for this sensor
  int newVal = digitalRead(blockSense);
  if (newVal != HIGH) {
    blockSenseActive = true;
    blockSenseState = !newVal;
    blockSenseChangeTime = millis();
  }  else if (millis() - blockSenseChangeTime > debounceDelay) {
    blockSenseActive = false;
    blockSenseChangeTime = 0;
    blockSenseState = !newVal;
  }
}

void readInputs() {
  // Read all inputs
  readAnInput(openA, openAState, openAChangeTime, openAConfTime, openAActive);
  readAnInput(openB, openBState, openBChangeTime, openBConfTime, openBActive);
  readBlockSense();
  lCurrentVal = analogRead(lCurrent);
  rCurrentVal = analogRead(rCurrent);
  leftStopVal = analogRead(leftStop);
  rightStopVal = analogRead(rightStop);
}

void startMoving() {
  // Start moving the gate
  if (moveState == STOPPED) {
    moveState = UNLOCKING;
    digitalWrite(unlockPin, HIGH);
    lastMoveStateTime = millis();
    timeToNextChange = lockTime;
    Serial.println("Starting to move, unlocking");
  }
  if (target == CLOSE_T) {
    Serial.println("Set dirState to closing");
    dirState = CLOSING;
  } else {
    Serial.println("Set dirState to opening");
    dirState = OPENING;
  }
}

void stopMoving() {
  // Stop moving the gate
  if (!anyStopSensorActive()) {
    locState = MID;
  }
  if (moveState != STOPPED) {
    moveState = STOPPING;
    digitalWrite(lMotor, LOW);
    digitalWrite(rMotor, LOW);
    lastMoveStateTime = millis();
    timeToNextChange = stopTime;
    rampLevel = 0;
    Serial.println("Stopping");
  }
}