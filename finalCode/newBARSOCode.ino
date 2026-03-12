#include <SparkFun_TB6612.h>
#include <math.h>

#define PWMA 15
#define AIN2 14
#define AIN1 13
#define STBY 12
#define BIN1 11
#define BIN2 10
#define PWMB 9

#define OFFSETA 1
#define OFFSETB 1

#define ENCODERPINA1 27
#define ENCODERPINA2 28
#define ENCODERPINB1 17
#define ENCODERPINB2 16

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

// Turn PID gains
float kPTurn = 3.2f;
float kITurn = 0.01f;
float kDTurn = 0.35f;

// Stop condition
const float DIST_TOLERANCE_CM = 0.5f;   // acceptable final error
const unsigned long SETTLE_TIME_MS = 150;

// Turn stop condition
const float TURN_TOLERANCE_DEG = 1.0f; // acceptable final error
const unsigned long TURN_SETTLE_TIME_MS = 150;

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

float wrapAngle180(float angleDeg) {
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
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

void turnToAnglePID(float targetTurnDegrees) {
  float startYaw = getYaw();
  float targetYaw = wrapAngle180(startYaw + targetTurnDegrees);

  float integral = 0.0f;
  float previousError = wrapAngle180(targetYaw - startYaw);
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

    float currentYaw = getYaw();
    float error = wrapAngle180(targetYaw - currentYaw);

    // PID terms
    integral += error * dt;

    // Anti-windup
    if (integral > 100.0f) integral = 100.0f;
    if (integral < -100.0f) integral = -100.0f;

    float derivative = (error - previousError) / dt;
    float pidOutput = kPTurn * error + kITurn * integral + kDTurn * derivative;

    int turnCmd = (int)fabs(pidOutput);

    if (turnCmd > 0 && turnCmd < MIN_MOTOR_CMD) {
      turnCmd = MIN_MOTOR_CMD;
    }
    if (turnCmd > MAX_MOTOR_CMD) {
      turnCmd = MAX_MOTOR_CMD;
    }

    int direction = (pidOutput >= 0) ? 1 : -1;

    // Tank turn: left and right motors run opposite directions
    int leftCmd  = -direction * turnCmd;
    int rightCmd =  direction * turnCmd;

    leftCmd  = constrain(leftCmd, -255, 255);
    rightCmd = constrain(rightCmd, -255, 255);

    // Stop logic: yaw error must stay inside tolerance briefly
    if (fabs(error) <= TURN_TOLERANCE_DEG) {
      if (inToleranceSince == 0) {
        inToleranceSince = currentTime;
      } else if (currentTime - inToleranceSince >= TURN_SETTLE_TIME_MS) {
        break;
      }
    } else {
      inToleranceSince = 0;
    }

    leftMotor.drive(leftCmd);
    rightMotor.drive(rightCmd);

    Serial.print("TargetYaw: ");
    Serial.print(targetYaw);
    Serial.print("  CurrentYaw: ");
    Serial.print(currentYaw);
    Serial.print("  Error: ");
    Serial.print(error);
    Serial.print("  PID: ");
    Serial.println(pidOutput);

    previousError = error;
    previousTime = currentTime;

    delay(10);  // ~100 Hz
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
    delay(200);

    driveToDistancePID(70.0f);  // drive 70 cm
    delay(500);

    turnToAnglePID(90.0f);      // turn 90 degrees
    delay(10000);
  }
}