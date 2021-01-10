#include <Servo.h>

// Pins.
#define echoPin 2
#define trigPin 3
#define switchPin 4
#define escSignalPin 9

// ESC as servo.
Servo esc;

// Ultrasonic sensor variables.
long duration = 0; // Duration of sound wave travel.
int distance = 0; // Distance measurement.

const int min_distance = 2; // Min and max measurable distance limit.
const int max_distance = 100;

// Switch variables.
int switchState = 0;

// Pot variables.
int pot_val = 0; // Potentiometer values (0-1023).

// PID variables.
const double proc_time = 0.6; // Constant process time.

double error = 0; // Errors.
double prev_error = 0;

double p = 0; // PID values.
double i = 0;
double d = 0;

double kp = 0.4; // PID factors values.
double ki = 0.005;
double kd = 0.03;

double integral = 0; // Temporary values for calculation.
double derivative = 0;

double total_val = 0; // Total PID value.

const int setpoint = 20; // Constant setpoint.

// Writing variables.
int write_val = 0; // Remapped value for writing to esc (manual / auto).

void setup() {
  // Start serial communication.
  Serial.begin(9600);

  // Define in and outputs.
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(switchPin, INPUT);

  // ESC startup.
  esc.attach(escSignalPin); // Attach.
  esc.writeMicroseconds(1500); // Send stop signal to ESC.
  delay(7000); // Allow ESC to recognize stop signal.
}

void loop() {
  switchState = digitalRead(switchPin); // Read switch state.

  // Compare switch states.
  if (switchState == HIGH) {
    pidControl(); // Auto mode (PID control).
  } else {
    potControl(); // Manual mode (potentiometer control).
  }
}

void pidControl() {
  // -- Ultrasonic sensor code. --
  digitalWrite(trigPin, LOW); // Clear trigPin.
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH); // trigPin HIGH.
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW); // trigPin LOW.

  duration = pulseIn(echoPin, HIGH); // Read echoPin (sound wave travel time).
  distance = duration * 0.034 / 2; // Calculate distance.

  // Limit distance values.
  if (distance > max_distance) {
    distance = max_distance;
  } else if (distance < min_distance) {
    distance = min_distance;
  }
  // -- End ultrasonic sensor code. --

  // -- PID code. --
  // Calculate error.
  error = setpoint - distance;

  // Proportional action.
  p = kp * error;

  // Integral action (anti-windup).
  if(i < -20){
    i = -20;
  }
  if(i > 20){
    i = 20;
  }

  if (!(i <= -20 && error < 0) || (i >= 20 && error > 0)) {
    integral = error * proc_time;
    i += ki * integral;
  }

  // Derivative action.
  derivative = (error - prev_error) / proc_time;
  d = kd * derivative;

  // Total.
  total_val = p + i + d;

  // Limit total value.
  if (total_val > 100) {
    total_val = 100;
  }
  if (total_val < 0) {
    total_val = 0;
  }

  prev_error = error; // Set previous error to current error.

  write_val = map(total_val, 0, 100, 0, 180); // Map 0-100 to 1100-1900 (servo us).
  esc.write(write_val); // Write value to esc.
  delay(10);

  // Print values.
  Serial.println();
  Serial.print("Switch state: " + String(switchState) + (" (auto mode)"));
  Serial.print("---");
  Serial.print("Setpoint: " + String(setpoint));
  Serial.print("---");
  Serial.print("Distance: " + String(distance));
  Serial.print("---");
  Serial.print("Error: " + String(error));
  Serial.print("---");
  Serial.print("P: " + String(p));
  Serial.print("---");
  Serial.print("I: " + String(i));
  Serial.print("---");
  Serial.print("D: " + String(d));
  Serial.print("---");
  Serial.print("Total value (0-100): " + String(total_val));
  Serial.print("---");
  Serial.print("Total => write value (0-180): " + String(write_val));
  Serial.print("---");
}

void potControl() {
  pot_val = analogRead(A0); // Read pot value.
  write_val = map(pot_val, 0, 1023, 0, 180); // Map from 0-1023 (pot) to 0-180 (servo us).
  esc.write(write_val); // Write value to esc.

  // Print values.
  Serial.println();
  Serial.print("---");
  Serial.print("Switch state: " + String(switchState) + (" (manual mode)"));
  Serial.print("---");
  Serial.print("Pot value (0-1023): " + String(pot_val));
  Serial.print("---");
  Serial.print("Pot => write value (0-180): " + String(write_val));
  Serial.print("---");
}
