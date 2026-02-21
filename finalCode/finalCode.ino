#include <SparkFun_TB6612.h>
#include <math.h>
// Left motor is A, right motor is B

#define PWMA 15
#define AIN2 14
#define AIN1 13
#define STBY 12
#define BIN1 11
#define BIN2 10
#define PWMB 9

#define OFFSETA 1
#define OFFSETB 1

#define ENCODERPINA2 28 
#define ENCODERPINA1 27
#define ENCODERPINB1 17
#define ENCODERPINB2 16

#define WHEELDIAMETERA 6.0
#define WHEELDIAMETERB 6.22
#define PULSESPERSHAFTREVOLUTION 1470.0

#define BUTTONPIN 2

Motor leftMotor = Motor(AIN1, AIN2, PWMA, OFFSETA, STBY);
Motor rightMotor = Motor(BIN1, BIN2, PWMB, OFFSETB, STBY);

float yaw;
volatile long pulseCountA = 0;
volatile long pulseCountB = 0;
volatile long distanceInCM = 0;

void countAPulse() {
  if (digitalRead(ENCODERPINA2) == HIGH) {
    pulseCountA++;
  } else {
    pulseCountA--;
  }
}

void countBPulse() {
  if (digitalRead(ENCODERPINB2) == HIGH) {
    pulseCountB++;
  } else {
    pulseCountB--;
  }
  Serial.println("Pulse count "+ String(pulseCountB) + "Pulse per shaft revolution "+ String(PULSESPERSHAFTREVOLUTION) + "Divided "+ String(pulseCountB/PULSESPERSHAFTREVOLUTION));
  distanceInCM = pulseCountB/PULSESPERSHAFTREVOLUTION*M_PI*WHEELDIAMETERB;
  Serial.println(distanceInCM);
}

// void driveForDistance(float distance, float speed) {
//   while (getDistance < distance) {
//     leftMotor.drive(speed);
//     rightMotor.drive(speed);
//   }
// }

void drive(float distance, float speed) {
  pulseCountB = 0;
  pulseCountA = 0;
  while (distanceInCM <= distance) {
    Serial.println(distanceInCM);
    yaw = getYaw();
    Serial.println(yaw);
    leftMotor.drive(-80);
    rightMotor.drive(-80);
    delay(1000);
    // Serial.println("Pulse count A: " + pulseCountA);
    // Serial.println("Pulse count B: " + pulseCountB);

    if (digitalRead(BUTTONPIN) == HIGH) {
      leftMotor.drive(0);
      rightMotor.drive(0);
      delay(1000);
      break;
    }
  }
  leftMotor.drive(0);
  rightMotor.drive(0);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(BUTTONPIN, INPUT);
  pinMode(ENCODERPINA1, INPUT_PULLUP);
  pinMode(ENCODERPINA2, INPUT_PULLUP);
  beginIMUCode();
  
  attachInterrupt(digitalPinToInterrupt(ENCODERPINA1), countAPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODERPINB1), countBPulse, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(BUTTONPIN) == HIGH) {
    pulseCountB = 0;
    distanceInCM = 0;
    drive(20, -150); // Don't go below 80; 1.75 secs for 20cm at -255, 
  }
}
