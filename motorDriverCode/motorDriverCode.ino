#include <SparkFun_TB6612.h>

#define AIN1 2
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

int totalDistance = 0;
int maxRPM = 140;
// A negative speed will cause it to go backwards.
// Speed can be from -255 to 255.

int startOfLoopTime = 0;

void setup() {
  //Nothing for now
}

void loop() {
  motor1.drive(-255);
  motor2.drive(255);
  delay(1000);
  motor1.brake();
  motor2.brake();

  startOfLoopTime = millis();
}

double getSpeed(PWM, maxRPM) {
  double pwm_percent = abs(PWM)/255;
  return ((pwm_percent * max_RPM)/60)*PI*6;
  //returns in cm/seconds

}