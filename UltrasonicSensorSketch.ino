#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor = AFMS.getMotor(1);

const int trigPin = 9;
const int echoPin = 10;

const int stopDist  = 10;
const int slowDist  = 25;
const int fullDist  = 50;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  if (!AFMS.begin()) {
    Serial.println("Shield not found!");
    while (1);
  }

  Serial.println("System ready!");
  motor->setSpeed(0);
  motor->run(RELEASE);
}

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

void loop() {
  long distance = getDistance();

  if (distance == -1) {
    Serial.println("No reading — motor stopped");
    motor->run(RELEASE);
    delay(200);
    return;
  }

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm  →  ");

  if (distance <= stopDist) {
    motor->setSpeed(0);
    motor->run(RELEASE);
    Serial.println("STOP");

  } else if (distance <= slowDist) {
    motor->setSpeed(120);
    motor->run(FORWARD);
    Serial.println("SLOW");

  } else if (distance <= fullDist) {
    motor->setSpeed(200);
    motor->run(FORWARD);
    Serial.println("MEDIUM");

  } else {
    motor->setSpeed(255);
    motor->run(FORWARD);
    Serial.println("FULL SPEED");
  }

  delay(200);
}
