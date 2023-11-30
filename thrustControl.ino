#include <Servo.h>
 
Servo servo1;

int thrustInPin = 2;
int thrustOutPin = 3;
int minThrust = 1600;
int maxThrust = 2000;
float minOutThrustVoltage = 0.9;
float maxOutThrustVoltage = 2.6;
int currentThrust;
int currentThrustVoltage;

int servoInPin = 4;
int servoOutPin = 5;
int minServo = 1150;
int zeroPositionServo;
int maxServo = 1930;
int currentServo;
int currentOutServo;

void setup() {
  Serial.begin(9600);
  delay(1000);
  zeroPositionServo = pulseIn(servoInPin, HIGH);
  pinMode(thrustOutPin, OUTPUT);
  servo1.attach(servoOutPin);
}

void loop() {
  configureThrust();
  configureServo();
  //delay(5);
}

void configureThrust() {
  currentThrust = pulseIn(thrustInPin, HIGH);
  currentThrustVoltage = getThrustVoltage(currentThrust);
  analogWrite(thrustOutPin, currentThrustVoltage);
  thrustLogger(currentThrust, currentThrustVoltage);
}

int getOutSignalFromVoltage(float voltage, bool withoutProcceed) {
  if(withoutProcceed) {
    return (255/5)*voltage;
  } else {
    return (255/5)*voltage * maxOutThrustVoltage/5;
  }
}

int getThrustVoltage(int currentThrust) {
  if(currentThrust < minThrust + 80)
    return getOutSignalFromVoltage(minOutThrustVoltage, true);

  currentThrust = currentThrust - 80;
  if(currentThrust > maxThrust)
    currentThrust = maxThrust;

  float percentage = (float)(currentThrust - minThrust)/(maxThrust - minThrust);

  return getOutSignalFromVoltage(minOutThrustVoltage, true) * (1 - percentage) + (255 * percentage * maxOutThrustVoltage/5);
}

void thrustLogger(int currentThrust, int currentThrustVoltage) {
  Serial.print("Thrust: Current duration: ");
  Serial.print(currentThrust);
  Serial.print("; Out signal: ");
  Serial.print(currentThrustVoltage); 
  Serial.print("; Out voltage: ");
  Serial.println(((float)currentThrustVoltage/255)*5);
}

void configureServo() {
  currentServo = pulseIn(servoInPin, HIGH);
  servo1.writeMicroseconds(currentServo);

/*
  if(currentServo == 0 || (currentServo <= zeroPositionServo + 40 && currentServo >= zeroPositionServo - 40)) {
    currentOutServo = 1500;
    servo1.writeMicroseconds(currentOutServo);
  }

  if(currentServo <= zeroPositionServo - 40) {
    currentOutServo = currentServo - (minServo - 1000) + 40;
    servo1.writeMicroseconds(currentOutServo);
  }

  if(currentServo >= zeroPositionServo + 40) {
    currentOutServo = currentServo + (2000 - maxServo) - 40;
    servo1.writeMicroseconds(currentOutServo);
  }*/

  Serial.print("Servo: Current In duration: ");
  Serial.print(currentServo);
  Serial.print("; Current Out duration: ");
  Serial.println(currentOutServo);
  currentOutServo = 0;
}