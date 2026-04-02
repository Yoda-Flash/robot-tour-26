// #include <SparkFun_TB6612.h>
// #include <math.h>
// // Left motor is A, right motor is B

// #define PWMA 15
// #define AIN2 14
// #define AIN1 13
// #define STBY 12
// #define BIN1 11
// #define BIN2 10
// #define PWMB 9

// #define OFFSETA 1
// #define OFFSETB 1

// #define ENCODERPINA2 28 
// #define ENCODERPINA1 27
// #define ENCODERPINB1 17
// #define ENCODERPINB2 16

// #define WHEELDIAMETERA 6.0
// #define WHEELDIAMETERB 6.0
// #define PULSESPERSHAFTREVOLUTION 1470.0

// #define BUTTONPIN 2

// Motor leftMotor = Motor(AIN1, AIN2, PWMA, OFFSETA, STBY);
// Motor rightMotor = Motor(BIN1, BIN2, PWMB, OFFSETB, STBY);

// volatile float yaw;
// volatile long pulseCountA = 0;
// volatile long pulseCountB = 0;
// volatile long distanceInCM = 0;

// float kP = 0;
// float kI = 0;
// float Kd = 0;

// float targetSpeed = 50;

// void countAPulse() {
//   if (digitalRead(ENCODERPINA2) == HIGH) {
//     pulseCountA++;
//   } else {
//     pulseCountA--;
//   }  
  
//   distanceInCM = pulseCountA/PULSESPERSHAFTREVOLUTION*M_PI*WHEELDIAMETERA;
//   Serial.print("Pulse count A:");
//   Serial.println(pulseCountA);
//   Serial.print("Distance: ");
//   Serial.println(distanceInCM);
// }

// void countBPulse() {
//   if (digitalRead(ENCODERPINB2) == HIGH) {
//     pulseCountB++;
//   } else {
//     pulseCountB--;
//   }
  
//   // distanceInCM = pulseCountB/PULSESPERSHAFTREVOLUTION*M_PI*WHEELDIAMETERB;
//   Serial.print("Pulse count B:");
//   Serial.println(pulseCountB);
//   Serial.print("Distance: ");
//   Serial.println(distanceInCM);
//   // return distanceInCM;
// }

// // void driveForDistance(float distance, float speed) {
// //   while (getDistance < distance) {
// //     leftMotor.drive(speed);
// //     rightMotor.drive(speed);
// //   }
// // }

// void driveForwards(float distance, float speed) {
//   pulseCountB = 0;
//   pulseCountA = 0;
//   while (distanceInCM <= distance) {
//     if (Serial.available() > 0){
//       distance = Serial.parseInt();
//     }
//     Serial.print("Driving, distance: ");
//     Serial.println(distanceInCM);
//     yaw = getYaw();
//     Serial.print("Yaw: ");
//     Serial.println(yaw);
//     leftMotor.drive(speed);
//     rightMotor.drive(speed);
//     delay(100);
//     // Serial.println("Pulse count A: " + pulseCountA);
//     // Serial.println("Pulse count B: " + pulseCountB);

//     if (digitalRead(BUTTONPIN) == HIGH) {
//       leftMotor.drive(0);
//       rightMotor.drive(0);
//       delay(1000);
//       break;
//     }
//   }
//   leftMotor.drive(0);
//   rightMotor.drive(0);
// }

// void driveBackwards(float distance, float speed) {
//   pulseCountB = 0;
//   pulseCountA = 0;
//   while (distanceInCM >= distance) {
//     Serial.print("Driving, distance: ");
//     Serial.println(distanceInCM);
//     yaw = getYaw();
//     Serial.print("Yaw: ");
//     Serial.println(yaw);
//     leftMotor.drive(speed);
//     rightMotor.drive(speed);
//     delay(100);
//     // Serial.println("Pulse count A: " + pulseCountA);
//     // Serial.println("Pulse count B: " + pulseCountB);

//     if (digitalRead(BUTTONPIN) == HIGH) {
//       leftMotor.drive(0);
//       rightMotor.drive(0);
//       delay(1000);
//       break;
//     }
//   }
//   leftMotor.drive(0);
//   rightMotor.drive(0);
// }

// void turn(float angleInCM, float speed) {
//   delay(100);
//   int startingMillis = millis();
//   pulseCountB = 0;
//   pulseCountA = 0;
//   Serial.print("Turning distance in CM");
//   Serial.println(distanceInCM);
//   while (abs(distanceInCM) <= abs(angleInCM)) {

//     Serial.println("Turning");
//     leftMotor.drive(-speed);
//     rightMotor.drive(speed);

//     delay(100);

//     if ((millis()-startingMillis) > 5200) {
//       leftMotor.drive(0);
//       rightMotor.drive(0);
//       delay(1000);
//       break;
//     }
    
//     if (digitalRead(BUTTONPIN) == HIGH) {
//       leftMotor.drive(0);
//       rightMotor.drive(0);
//       delay(1000);
//       break;
//     }
//   }
  
//   leftMotor.drive(0);
//   rightMotor.drive(0);

// }

// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(9600);
//   pinMode(BUTTONPIN, INPUT);
//   pinMode(ENCODERPINA1, INPUT_PULLUP);
//   pinMode(ENCODERPINA2, INPUT_PULLUP);
//   beginIMUCode();
  
//   attachInterrupt(digitalPinToInterrupt(ENCODERPINA1), countAPulse, RISING);
//   attachInterrupt(digitalPinToInterrupt(ENCODERPINB1), countBPulse, RISING);
// }

// void run() {
//   delay(200);
//     pulseCountA = 0;
//     pulseCountB = 0;
//     distanceInCM = 0;
//     driveForwards(25, 250);
//   delay(200);
//     pulseCountA = 0;
//     pulseCountB = 0;
//     distanceInCM = 0;
//     turn(14, 50);
    
//   delay(200);
//     pulseCountA = 0;
//     pulseCountB = 0;
//     distanceInCM = 0;
//     driveForwards(5*25, 250);
    
//   delay(200);
//     pulseCountA = 0;
//     pulseCountB = 0;
//     distanceInCM = 0;
//     driveBackwards(-8*25, 250);
    
//   delay(200);
//     pulseCountA = 0;
//     pulseCountB = 0;
//     distanceInCM = 0;
//     turn(-14, 50);
    
//   delay(200);
//     pulseCountA = 0;
//     pulseCountB = 0;
//     distanceInCM = 0;
//     driveForwards(8*25, 250);
    
//   delay(200);
//     pulseCountA = 0;
//     pulseCountB = 0;
//     distanceInCM = 0;
//     turn(14, 50);
    
//   delay(200);
//     pulseCountA = 0;
//     pulseCountB = 0;
//     distanceInCM = 0;
//     driveForwards(25*2, 250);

    
//   delay(200);
//     pulseCountA = 0;
//     pulseCountB = 0;
//     distanceInCM = 0;
//     turn(14, 50);

//     // delay(200);
//     // pulseCountA = 0;
//     // pulseCountB = 0;
//     // distanceInCM = 0;
//     // turn(targetRotation, 50); 
//     // pulseCountA = 0;
//     // pulseCountB = 0;
//     // distanceInCM = 0;
//     // turn(15.0, -50); 
//     // delay(200);
//     // pulseCountA = 0;
//     // pulseCountB = 0;
//     // distanceInCM = 0;
//     // driveBackwards(-25*sqrt(2), -150);   
// }

// void loop() {
//   // put your main code here, to run repeatedly:

    
//     if (Serial.available() > 0){
//       targetSpeed = Serial.parseInt();
//       Serial.println("Changed distance");
//     }
//   if (digitalRead(BUTTONPIN) == HIGH) {
//     run();
//     delay(2000);
//     // pulseCountA = 0;
//     // pulseCountB = 0;
//     // distanceInCM = 0;
//     // drive(30, 150); // Don't go below 14; 1.75 secs for 20cm at -255, 
//   }
// }
