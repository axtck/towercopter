const int buttonPin = 4;

int buttonState = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  buttonState = digitalRead(buttonPin);

  if(buttonState == HIGH){
    Serial.println("High");
  } else {
    Serial.println("Low");
  }

  delay(500);
}
