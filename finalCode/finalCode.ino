#include <SparkFun_TB6612.h>
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

#define BUTTONPIN 2

Motor leftMotor = Motor(AIN1, AIN2, PWMA, OFFSETA, STBY);
Motor rightMotor = Motor(BIN1, BIN2, PWMB, OFFSETB, STBY);

void drive() {
  Serial.println("Button pressed");
  while (true) {
    leftMotor.drive(-255);
    rightMotor.drive(-255);
    delay(1000);

    if (digitalRead(BUTTONPIN) == HIGH) {
      leftMotor.drive(0);
      rightMotor.drive(0);
      break;
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(BUTTONPIN, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(BUTTONPIN) == HIGH) {
    Serial.println("Button pressed!");
    drive();
  }
}
