// Main program file for towercopter.

// Includes.
#include <Servo.h>

// Pins.
#define echoPin 2
#define trigPin 3
#define switchPin 4
#define escSignalPin 9

// ESC as servo.
Servo esc;

// Ultrasonic sensor variables.
long duration; // Var for the duration of sound wave travel.
int distance; // Var for the distance measurement.

// Button variables.
int switchState = 0;

// Pot variables.
int potValue;
int potEscValue;

// Motor constants.
const int MIN = 1000;
const int MAX = 1800;

// PID variables.
double timelatest = 0.6;
double timelast = 0;
double error = 0;
double errorlast = 0;
double p = 0;
double i = 0;
double d = 0;
double out_val;
float kp = 0.005;
float ki = 0.0003;
float kd = 0.039;
double nlast = 0;
double setpoint = 0;
double pv = 0;
int mv = 0;
double out = 0;
double input = 0;
String value = "";


void setup() {
  // Start serial communication.
  Serial.begin(9600);

  // Define as in or output.
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(switchPin, INPUT);

  // Attach.
  esc.attach(escSignalPin, MIN, MAX);
  esc.writeMicroseconds(0);
}

void loop() {
  // Read switch state.
  switchState = digitalRead(switchPin);

  // Printing on different button modes.
  if (switchState == HIGH) {
    // Auto mode.
    pidControl();
  } else {
    // Manual mode.
    potControl();
  }
}

void pidControl() {
  // Clears the trigPin condition.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH); // trigPin HIGH.
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW); // trigPin LOW.

  // Reads the echoPin, returns the sound wave travel time in microseconds.
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back).

  input = map(distance, 2, 50, 0, 100);

  error = setpoint - input;
  p = (kp * error);
  i = ki * ((timelatest * errorlast) + (timelatest * ((error - errorlast) / 2)));
  d = kd * ((error - errorlast) / timelatest);
  out_val = p + i + d;
  out += out_val + nlast;

  if (out > 100) {
    out = 100;
  }
  if (out < 0) {
    out = 0;
  }
  errorlast = error;
  nlast = i;
  mv = map(out, 0, 100, 0, 180);
  esc.write(mv);
  delay(10);

  setpoint = 50;

  Serial.println();
  Serial.print("Switch state: " + String(switchState) + (" (auto mode)"));
  Serial.print("---");
  Serial.print("Distance: " + String(distance));
  Serial.print("---");
  Serial.print("Input: " + String(input));
  Serial.print("---");
  Serial.print("Out: " + String(out));
  Serial.print("---");
  Serial.print("Mv: " + String(mv));
  Serial.print("---");
}

void potControl() {
  potValue = analogRead(A0);
  potEscValue = map(potValue, 0, 1023, 0, 180); // Map from 0-1023 (pot) to 0-180 (servo).
  esc.write(potEscValue);

  Serial.println();
  Serial.print("---");
  Serial.print("Switch state: " + String(switchState) + (" (manual mode)"));
  Serial.print("---");
  Serial.print("Pot value (0-1023): " + String(potValue));
  Serial.print("---");
  Serial.print("Pot => ESC value (0-180): " + String(potEscValue));
  Serial.print("---");
}
