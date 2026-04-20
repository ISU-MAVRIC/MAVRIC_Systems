#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *leftMotor       = AFMS.getMotor(3);
Adafruit_DCMotor *rightMotor      = AFMS.getMotor(4);
Adafruit_DCMotor *rightMotorfront = AFMS.getMotor(2);
Adafruit_DCMotor *leftMotorfront  = AFMS.getMotor(1);

// ─── Pins ─────────────────────────────────────────────
const int trigPin = 9;
const int echoPin = 10;
const int IR      = 11;   // IR receiver data pin (ESP32)
const int stopDist = 15;  // cm

// ─── Button to use ────────────────────────────────────
#define YOUR_BUTTON_CODE 0xFF30CF  // Button 1 — change if needed

// ─── State ────────────────────────────────────────────
bool running = false;

// ─── IR: raw NEC decoder ──────────────────────────────
unsigned long readPulse(int level, unsigned long timeout = 20000) {
  unsigned long start = micros();
  while (digitalRead(IR) == level) {
    if (micros() - start > timeout) return 0;
  }
  return micros() - start;
}

// Returns decoded NEC code, or 0 if nothing received / invalid
unsigned long readIR() {
  if (digitalRead(IR) != LOW) return 0;  // Nothing incoming

  unsigned long lowTime = readPulse(LOW);
  if (lowTime < 8000 || lowTime > 10000) return 0;  // Leader LOW ~9ms

  unsigned long highTime = readPulse(HIGH);
  if (highTime < 4000 || highTime > 5000) return 0;  // Leader HIGH ~4.5ms

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

// ─── IR check ─────────────────────────────────────────
void checkRemote() {
  unsigned long code = readIR();
  if (code == 0 || code == 0xFFFFFFFF) return;  // Nothing or repeat — ignore

  if (code == YOUR_BUTTON_CODE) {
    running = !running;
    if (!running) {
      stopAll();
      Serial.println(">> Remote: STOPPED");
    } else {
      Serial.println(">> Remote: STARTED");
    }
    delay(150);  // Debounce
  }
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

// ─── Safe delay ───────────────────────────────────────
void safeDelay(int durationMs) {
  int elapsed = 0;
  while (elapsed < durationMs) {
    checkRemote();

    if (!running) {
      stopAll();
      return;
    }

    if (objectDetected()) {
      stopAll();
      Serial.println(">> Object detected! Pausing.");
      while (objectDetected() && running) {
        checkRemote();
        delay(100);
      }
      if (running) Serial.println(">> Clear! Resuming.");
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
    Serial.println(">> Sequence complete. Press button to run again.");
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
  Serial.println("Press button 1 on remote to start!");
}

void loop() {
  checkRemote();  // Always listening

  if (running) {
    runSequence();
  }
}
  
