// Main program file for towercopter.

// Includes.
#include <Servo.h>

// Pins.
#define echoPin 2
#define trigPin 3
#define buttonPin 4
#define escSignalPin 9

// ESC as servo.
Servo esc;

// Ultrasonic sensor variables.
long duration; // Var for the duration of sound wave travel.
int distance; // Var for the distance measurement.
const int num_distances = 20;
int distances[num_distances];
int index = 0;
int total = 0;
int average = 0;

// Button variables.
int buttonState = 0;

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
  pinMode(buttonPin, INPUT);

  // Attach.
  esc.attach(escSignalPin, MIN, MAX);
  esc.writeMicroseconds(0);
}

void loop() {
  // Clears the trigPin condition.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH); // trigPin HIGH.
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW); // trigPin LOW.


  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) {
    Serial.println("Auto mode");
  } else {
    Serial.println("Manual mode");
  }


  // Reads the echoPin, returns the sound wave travel time in microseconds.
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back).
  /*total = total - distances[index];
    distances[index] = distance;
    total = total + distances[index];
    index = index + 1;
    if(index >= num_distances){
    index = 0;
    }

    average = total/num_distances;

    input = map(average, 2, 50, 0, 100);*/

  input = map(distance, 2, 50, 0, 100);

  pid();
  mv = map(out, 0, 100, 0, 180);
  esc.write(mv);
  delay(10);

  setpoint = 50;

}

void pid() {
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
}
