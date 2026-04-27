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
const int stopDist = 15;
const int turnTime = 600;

// ─── Button codes ─────────────────────────────────────
#define START_STOP_BTN  0xFF30CF  // Button 1 — start/stop sequence
#define PAUSE_BTN       0xFF02FD  // Pause — pause/resume sequence
#define MODE_SWITCH_BTN 0xFF6897  // Button 0 — toggle manual/sequence mode
#define BTN_UP          0xFF18E7  // Button 2 — forward
#define BTN_LEFT        0xFF10EF  // Button 4 — turn left
#define BTN_RIGHT       0xFF5AA5  // Button 6 — turn right
#define BTN_DOWN        0xFF4AB5  // Button 8 — backward
#define BTN_STOP        0xFF38C7  // Button 5 — stop

// ─── Mode ─────────────────────────────────────────────
enum Mode { SEQUENCE_MODE, MANUAL_MODE };
Mode currentMode = SEQUENCE_MODE;

// ─── State ────────────────────────────────────────────
bool running       = false;
bool paused        = false;
bool manualMode    = false;
int  manualTimeout = 0;

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
  Serial.println(">> Obstacle! Scanning for clear path...");
  stopAll();
  delay(300);

  // Scan right
  setMotors(150, 150, FORWARD, BACKWARD);
  delay(400);
  stopAll();
  delay(100);
  long rightDist = getDistance();
  Serial.print(">> Right distance: "); Serial.println(rightDist);

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
  Serial.print(">> Left distance: "); Serial.println(leftDist);

  // Return to centre
  setMotors(150, 150, FORWARD, BACKWARD);
  delay(400);
  stopAll();
  delay(100);

  if (rightDist == -1) rightDist = 0;
  if (leftDist  == -1) leftDist  = 0;

  if (rightDist >= leftDist) {
    Serial.println(">> Turning RIGHT — clearer path");
    setMotors(180, 180, FORWARD, BACKWARD);
  } else {
    Serial.println(">> Turning LEFT — clearer path");
    setMotors(180, 180, BACKWARD, FORWARD);
  }
  delay(turnTime);
  stopAll();
  delay(200);

  if (objectDetected()) {
    Serial.println(">> Still blocked — trying opposite direction");
    if (rightDist >= leftDist) {
      setMotors(180, 180, BACKWARD, FORWARD);
    } else {
      setMotors(180, 180, FORWARD, BACKWARD);
    }
    delay(turnTime * 2);
    stopAll();
    delay(200);
  }

  Serial.println(">> Path clear — resuming sequence.");
}

// ─── Manual steering ──────────────────────────────────
void handleManual(unsigned long code) {
  switch (code) {
    case BTN_UP:
      Serial.println(">> Manual: FORWARD");
      setMotors(200, 200, FORWARD, FORWARD);
      manualTimeout = 300;
      break;
    case BTN_DOWN:
      Serial.println(">> Manual: BACKWARD");
      setMotors(200, 200, BACKWARD, BACKWARD);
      manualTimeout = 300;
      break;
    case BTN_LEFT:
      Serial.println(">> Manual: TURN LEFT");
      setMotors(200, 200, BACKWARD, FORWARD);
      manualTimeout = 300;
      break;
    case BTN_RIGHT:
      Serial.println(">> Manual: TURN RIGHT");
      setMotors(200, 200, FORWARD, BACKWARD);
      manualTimeout = 300;
      break;
    case BTN_STOP:
      Serial.println(">> Manual: STOP");
      stopAll();
      manualTimeout = 0;
      break;
  }
}

// ─── IR check ─────────────────────────────────────────
void checkRemote() {
  unsigned long code = readIR();
  if (code == 0) return;

  // Repeat code — extend manual movement if holding button
  if (code == 0xFFFFFFFF) {
    if (currentMode == MANUAL_MODE) manualTimeout = 300;
    return;
  }

  // ── Mode switch — always available ──────────────────
  if (code == MODE_SWITCH_BTN) {
    stopAll();
    running       = false;
    paused        = false;
    manualTimeout = 0;

    if (currentMode == SEQUENCE_MODE) {
      currentMode = MANUAL_MODE;
      Serial.println(">> Switched to MANUAL MODE — use 2/4/6/8 to steer");
    } else {
      currentMode = SEQUENCE_MODE;
      Serial.println(">> Switched to SEQUENCE MODE — press 1 to start");
    }
    delay(150);
    return;
  }

  // ── Sequence mode controls ───────────────────────────
  if (currentMode == SEQUENCE_MODE) {
    if (code == START_STOP_BTN) {
      running = !running;
      paused  = false;
      if (!running) {
        stopAll();
        Serial.println(">> Sequence: STOPPED");
      } else {
        Serial.println(">> Sequence: STARTED");
      }
      delay(150);
      return;
    }

    if (code == PAUSE_BTN) {
      if (!running) return;
      paused = !paused;
      if (paused) {
        stopAll();
        Serial.println(">> Sequence: PAUSED");
      } else {
        Serial.println(">> Sequence: RESUMED");
      }
      delay(150);
      return;
    }
  }

  // ── Manual mode controls ─────────────────────────────
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

  if (running) {
    stopAll();
    running = false;
    Serial.println(">> Sequence complete. Press 1 to run again.");
  }
}

// ─── Setup & Loop ─────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Serial.println("Robot ready!");

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(IR, INPUT);

  if (!AFMS.begin()) {
    Serial.println("Shield not found! Check wiring.");
    while (1);
  }

  stopAll();
  Serial.println("Press 0 to switch modes | Default: SEQUENCE MODE");
  Serial.println("Sequence: 1=Start/Stop, Pause=Pause");
  Serial.println("Manual:   2=Fwd, 8=Back, 4=Left, 6=Right, 5=Stop");
}

void loop() {
  checkRemote();

  // ── Manual mode loop ──────────────────────────────
  if (currentMode == MANUAL_MODE) {
    if (manualTimeout > 0) {
      manualTimeout -= 10;
    } else {
      stopAll();
    }
  }

  // ── Sequence mode loop ────────────────────────────
  if (currentMode == SEQUENCE_MODE && running && !paused) {
    runSequence();
  }
}