#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *leftMotor       = AFMS.getMotor(3);
Adafruit_DCMotor *rightMotor      = AFMS.getMotor(4);
Adafruit_DCMotor *rightMotorfront = AFMS.getMotor(2);
Adafruit_DCMotor *leftMotorfront  = AFMS.getMotor(1);

// ─── Pins ─────────────────────────────────────────────
const int trigPin  = 9;
const int echoPin  = 10;
const int IR       = 11;
const int stopDist = 15;   // cm — obstacle threshold
const int turnTime = 600;  // ms — how long to turn when avoiding

// ─── Button codes ─────────────────────────────────────
#define BTN_0     0xFF6897  // Mode switch
#define BTN_1     0xFF30CF  // Sequence start/stop
#define BTN_2     0xFF18E7  // Forward
#define BTN_3     0xFF7A85  // (unused)
#define BTN_4     0xFF10EF  // Turn left
#define BTN_5     0xFF38C7  // Stop
#define BTN_6     0xFF5AA5  // Turn right
#define BTN_8     0xFF4AB5  // Backward
#define BTN_PAUSE 0xFF02FD  // Pause/resume sequence

// ─── Mode ─────────────────────────────────────────────
enum Mode { SEQUENCE_MODE, MANUAL_MODE };
Mode currentMode = SEQUENCE_MODE;

// ─── State ────────────────────────────────────────────
bool running        = false;
bool paused         = false;
unsigned long manualLastPress = 0;  // Timestamp of last manual button press
const int manualHoldTime = 300;     // ms motors run after button press

// ─── IR: raw NEC decoder ──────────────────────────────
unsigned long readPulse(int level, unsigned long timeout = 20000) {
  unsigned long start = micros();
  while (digitalRead(IR) == level) {
    if (micros() - start > timeout) return 0;
  }
  return micros() - start;
}

unsigned long readIR() {
  if (digitalRead(IR) != LOW) return 0;

  unsigned long lowTime = readPulse(LOW);
  if (lowTime < 8000 || lowTime > 10000) return 0;

  unsigned long highTime = readPulse(HIGH);
  if (highTime < 4000 || highTime > 5000) return 0;

  unsigned long code = 0;
  for (int i = 0; i < 32; i++) {
    unsigned long bitLow = readPulse(LOW);
    if (bitLow < 400 || bitLow > 700) return 0;

    unsigned long bitHigh = readPulse(HIGH);
    if (bitHigh == 0) return 0;

    code <<= 1;
    if (bitHigh > 1000) code |= 1;
  }
  return code;
}

// ─── Motor helpers ────────────────────────────────────
void setMotors(int lSpeed, int rSpeed, int lCmd, int rCmd) {
  leftMotor->setSpeed(lSpeed);       leftMotor->run(lCmd);
  leftMotorfront->setSpeed(lSpeed);  leftMotorfront->run(lCmd);
  rightMotor->setSpeed(rSpeed);      rightMotor->run(rCmd);
  rightMotorfront->setSpeed(rSpeed); rightMotorfront->run(rCmd);
}

void stopAll() {
  setMotors(0, 0, RELEASE, RELEASE);
}

// ─── Sensor ───────────────────────────────────────────
long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.034 / 2;
}

bool objectDetected() {
  long d = getDistance();
  return (d != -1 && d <= stopDist);
}

// ─── Obstacle avoidance ───────────────────────────────
void avoidObstacle() {
  Serial.println(">> Obstacle! Scanning...");
  stopAll();
  delay(300);

  // Scan right
  setMotors(150, 150, FORWARD, BACKWARD);
  delay(400);
  stopAll();
  delay(100);
  long rightDist = getDistance();
  Serial.print(">> Right: "); Serial.println(rightDist);

  // Return to centre
  setMotors(150, 150, BACKWARD, FORWARD);
  delay(400);
  stopAll();
  delay(100);

  // Scan left
  setMotors(150, 150, BACKWARD, FORWARD);
  delay(400);
  stopAll();
  delay(100);
  long leftDist = getDistance();
  Serial.print(">> Left: "); Serial.println(leftDist);

  // Return to centre
  setMotors(150, 150, FORWARD, BACKWARD);
  delay(400);
  stopAll();
  delay(100);

  if (rightDist == -1) rightDist = 0;
  if (leftDist  == -1) leftDist  = 0;

  // Turn toward clearer side
  if (rightDist >= leftDist) {
    Serial.println(">> Turning RIGHT");
    setMotors(180, 180, FORWARD, BACKWARD);
  } else {
    Serial.println(">> Turning LEFT");
    setMotors(180, 180, BACKWARD, FORWARD);
  }
  delay(turnTime);
  stopAll();
  delay(200);

  // Still blocked — try opposite side
  if (objectDetected()) {
    Serial.println(">> Still blocked — trying other side");
    if (rightDist >= leftDist) {
      setMotors(180, 180, BACKWARD, FORWARD);
    } else {
      setMotors(180, 180, FORWARD, BACKWARD);
    }
    delay(turnTime * 2);
    stopAll();
    delay(200);
  }

  Serial.println(">> Clear — resuming.");
}

// ─── Manual steering ──────────────────────────────────
void handleManual(unsigned long code) {
  switch (code) {
    case BTN_2:
      Serial.println(">> FORWARD");
      setMotors(200, 200, FORWARD, FORWARD);
      manualLastPress = millis();
      break;
    case BTN_8:
      Serial.println(">> BACKWARD");
      setMotors(200, 200, BACKWARD, BACKWARD);
      manualLastPress = millis();
      break;
    case BTN_4:
      Serial.println(">> LEFT");
      setMotors(200, 200, BACKWARD, FORWARD);
      manualLastPress = millis();
      break;
    case BTN_6:
      Serial.println(">> RIGHT");
      setMotors(200, 200, FORWARD, BACKWARD);
      manualLastPress = millis();
      break;
    case BTN_5:
      Serial.println(">> STOP");
      stopAll();
      manualLastPress = 0;
      break;
  }
}

// ─── IR check ─────────────────────────────────────────
void checkRemote() {
  unsigned long code = readIR();
  if (code == 0) return;

  // Repeat code — keep moving in manual mode
  if (code == 0xFFFFFFFF) {
    if (currentMode == MANUAL_MODE && manualLastPress > 0) {
      manualLastPress = millis();
    }
    return;
  }

  Serial.print("Button: 0x"); Serial.println(code, HEX);

  // ── Mode switch — always available ──────────────────
  if (code == BTN_0) {
    stopAll();
    running         = false;
    paused          = false;
    manualLastPress = 0;

    if (currentMode == SEQUENCE_MODE) {
      currentMode = MANUAL_MODE;
      Serial.println(">> MANUAL MODE — 2=Fwd 8=Back 4=Left 6=Right 5=Stop");
    } else {
      currentMode = SEQUENCE_MODE;
      Serial.println(">> SEQUENCE MODE — press 1 to start");
    }
    delay(300);  // Increased from 150 to 300ms to prevent double trigger
    return;
  }

  // ── Sequence mode ────────────────────────────────────
  if (currentMode == SEQUENCE_MODE) {
    if (code == BTN_1) {
      running = !running;
      paused  = false;
      if (!running) {
        stopAll();
        Serial.println(">> Sequence STOPPED");
      } else {
        Serial.println(">> Sequence STARTED");
      }
      delay(150);
    }
    else if (code == BTN_PAUSE) {
      if (!running) return;
      paused = !paused;
      if (paused) {
        stopAll();
        Serial.println(">> Sequence PAUSED");
      } else {
        Serial.println(">> Sequence RESUMED");
      }
      delay(150);
    }
    return;
  }

  // ── Manual mode ──────────────────────────────────────
  if (currentMode == MANUAL_MODE) {
    handleManual(code);
  }
}

// ─── Safe delay ───────────────────────────────────────
void safeDelay(int durationMs) {
  int elapsed = 0;
  while (elapsed < durationMs) {
    checkRemote();

    if (!running || currentMode != SEQUENCE_MODE) {
      stopAll();
      return;
    }

    while (paused && running) {
      checkRemote();
      delay(50);
    }

    if (objectDetected()) {
      avoidObstacle();
    }

    delay(50);
    elapsed += 50;
  }
}

// ─── Movement sequence ────────────────────────────────
void runSequence() {
  if (!running) return;

  Serial.println("Step 1: Backward 5s");
  setMotors(100, 100, BACKWARD, BACKWARD);
  safeDelay(5000);

  if (!running) return;
  Serial.println("Step 2: Forward 5s");
  setMotors(100, 100, FORWARD, FORWARD);
  safeDelay(5000);

  if (!running) return;
  Serial.println("Step 3: Turn right 5s");
  setMotors(100, 100, FORWARD, BACKWARD);
  safeDelay(5000);

  if (!running) return;
  Serial.println("Step 4: Turn left 5s");
  setMotors(100, 100, BACKWARD, FORWARD);
  safeDelay(5000);

  if (!running) return;
  Serial.println("Step 5: Curve backward 5s");
  setMotors(150, 100, BACKWARD, BACKWARD);
  safeDelay(5000);

stopAll();
  running = false;
  paused  = false;
  Serial.println(">> Sequence complete. Press 1 to run again.");
  return;  // Return immediately so loop() can check remote again
  }


// ─── Setup ────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(IR, INPUT);

  if (!AFMS.begin()) {
    Serial.println("Shield not found! Check wiring.");
    while (1);
  }

  stopAll();
  Serial.println("Robot ready!");
  Serial.println("0=Switch mode | Default: SEQUENCE MODE");
  Serial.println("Sequence: 1=Start/Stop  Pause=Pause/Resume");
  Serial.println("Manual:   2=Fwd 8=Back 4=Left 6=Right 5=Stop");
}

// ─── Loop ─────────────────────────────────────────────
void loop() {
  checkRemote();

  // Manual mode — auto stop after holdTime ms with no button press
  if (currentMode == MANUAL_MODE) {
    if (manualLastPress > 0 && millis() - manualLastPress > manualHoldTime) {
      stopAll();
      manualLastPress = 0;
    }
  }

  // Sequence mode
 if (currentMode == SEQUENCE_MODE && running && !paused) {
    runSequence();
    return;  // Exit loop immediately after sequence ends so remote is checked
  }
}