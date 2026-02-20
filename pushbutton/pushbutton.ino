int buttonPin = 2;

void buttonActivated() {
  Serial.println("Pushbutton pressed!");
  delay(1000);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(buttonPin) == HIGH) {
    buttonActivated();
  }
}
