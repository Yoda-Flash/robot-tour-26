#include <SparkFun_TB6612.h>
#include <math.h>

// -------------------- Motor pins --------------------
#define PWMA 15
#define AIN2 14
#define AIN1 13
#define STBY 12
#define BIN1 11
#define BIN2 10
#define PWMB 9

#define OFFSETA 1
#define OFFSETB 1

// -------------------- Encoder pins --------------------
#define ENCODERPINA1 27
#define ENCODERPINA2 28
#define ENCODERPINB1 17
#define ENCODERPINB2 16

// -------------------- Robot constants --------------------
#define WHEELDIAMETERA 6.00f   // cm
#define WHEELDIAMETERB 6.22f   // cm
#define PULSESPERSHAFTREVOLUTION 1470.0f

#define BUTTONPIN 2

#define FORWARD_SIGN 1

Motor leftMotor  = Motor(AIN1, AIN2, PWMA, OFFSETA, STBY);
Motor rightMotor = Motor(BIN1, BIN2, PWMB, OFFSETB, STBY);

// -------------------- Globals --------------------
volatile long pulseCountA = 0;
volatile long pulseCountB = 0;

float yaw = 0.0f;

// Distance PID gains
float kP = 8.0f;
float kI = 0.08f;
float kD = 2.0f;

// Optional heading correction gain
float kYaw = 2.0f;

// Stop condition
const float DIST_TOLERANCE_CM = 0.5f;   // acceptable final error
const unsigned long SETTLE_TIME_MS = 150;

// Motor command limits
const int MAX_MOTOR_CMD = 180;
const int MIN_MOTOR_CMD = 40; // possibly lower depending on how high the friction is

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
}

float getDistanceCmA() {
  long countA;
  noInterrupts();
  countA = pulseCountA;
  interrupts();

  return (countA / PULSESPERSHAFTREVOLUTION) * (M_PI * WHEELDIAMETERA);
}

float getDistanceCmB() {
  long countB;
  noInterrupts();
  countB = pulseCountB;
  interrupts();

  return (countB / PULSESPERSHAFTREVOLUTION) * (M_PI * WHEELDIAMETERB);
}

float getAverageDistanceCm() {
  return 0.5f * (getDistanceCmA() + getDistanceCmB());
}

void resetEncoders() {
  noInterrupts();
  pulseCountA = 0;
  pulseCountB = 0;
  interrupts();
}

void stopMotors() {
  leftMotor.drive(0);
  rightMotor.drive(0);
}

void driveToDistancePID(float targetDistanceCm) {
  resetEncoders();

  float startYaw = getYaw();   // hold initial heading
  float integral = 0.0f;
  float previousError = targetDistanceCm;
  unsigned long previousTime = millis();
  unsigned long inToleranceSince = 0;

  while (true) {
    if (digitalRead(BUTTONPIN) == HIGH) {
      stopMotors();
      delay(300);
      return;
    }

    unsigned long currentTime = millis();
    float dt = (currentTime - previousTime) / 1000.0f;

    if (dt <= 0.0f) {
      dt = 0.001f;
    }

    float distanceCm = getAverageDistanceCm();
    float error = targetDistanceCm - distanceCm;

    // PID terms
    integral += error * dt;

    // Basic anti-windup
    if (integral > 50.0f) integral = 50.0f;
    if (integral < -50.0f) integral = -50.0f;

    float derivative = (error - previousError) / dt;

    float pidOutput = kP * error + kI * integral + kD * derivative;

    // Convert PID output to motor command magnitude
    int baseCmd = (int)fabs(pidOutput);

    if (baseCmd > 0 && baseCmd < MIN_MOTOR_CMD) {
      baseCmd = MIN_MOTOR_CMD;
    }
    if (baseCmd > MAX_MOTOR_CMD) {
      baseCmd = MAX_MOTOR_CMD;
    }

    // Determine direction from sign of PID output
    int direction = (pidOutput >= 0) ? FORWARD_SIGN : -FORWARD_SIGN;

    // Heading correction
    yaw = getYaw();
    float yawError = startYaw - yaw;
    int correction = (int)(kYaw * yawError);

    int leftCmd  = direction * (baseCmd - correction);
    int rightCmd = direction * (baseCmd + correction);

    leftCmd  = constrain(leftCmd, -255, 255);
    rightCmd = constrain(rightCmd, -255, 255);

    // Stop logic: distance must stay inside tolerance briefly
    if (fabs(error) <= DIST_TOLERANCE_CM) {
      if (inToleranceSince == 0) {
        inToleranceSince = currentTime;
      } else if (currentTime - inToleranceSince >= SETTLE_TIME_MS) {
        break;
      }
    } else {
      inToleranceSince = 0;
    }

    leftMotor.drive(leftCmd);
    rightMotor.drive(rightCmd);

    // Debug prints in the main loop only, never in the ISR
    Serial.print("Dist: ");
    Serial.print(distanceCm);
    Serial.print("  Error: ");
    Serial.print(error);
    Serial.print("  PID: ");
    Serial.print(pidOutput);
    Serial.print("  Yaw: ");
    Serial.println(yaw);

    previousError = error;
    previousTime = currentTime;

    delay(10);  // ~100 Hz control loop
  }

  stopMotors();
}

// -------------------- Arduino setup/loop --------------------
void setup() {
  Serial.begin(115200);

  pinMode(BUTTONPIN, INPUT);

  pinMode(ENCODERPINA1, INPUT_PULLUP);
  pinMode(ENCODERPINA2, INPUT_PULLUP);
  pinMode(ENCODERPINB1, INPUT_PULLUP);
  pinMode(ENCODERPINB2, INPUT_PULLUP);

  beginIMUCode();

  attachInterrupt(digitalPinToInterrupt(ENCODERPINA1), countAPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODERPINB1), countBPulse, RISING);
}

void loop() {
  if (digitalRead(BUTTONPIN) == HIGH) {
    pulseCountB = 0;
    distanceInCM = 0;

    delay(200);                 // debounce
    driveToDistancePID(70.0f);  // drive 70 cm
    delay(10000);
  }
}