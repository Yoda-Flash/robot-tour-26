const int encoderPinA = 2; 
const int encoderPinB = 3;
volatile long pulseCount = 0;

void setup() {
  Serial.begin(9600);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  
  // Trigger count pulse on every change
  attachInterrupt(digitalPinToInterrupt(encoderPinA), countPulse, RISING);
}

void loop() {
  Serial.print("Wheel turns: ");
  Serial.println(getWheelTurns());
  delay(100);
}

void countPulse() {
  if (digitalRead(encoderPinB) == HIGH) {
    pulseCount++;
  } else {
    pulseCount--;
  }
}

int getWheelTurns() {
  return (pulseCount)/(1400); //7 ppr, reduction ration of 200
}