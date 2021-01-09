#include <Filters.h>

// Pins.
#define echoPin 2
#define trigPin 3

// Ultrasonic sensor variables.
long duration; // Var for the duration of sound wave travel.
long distance; // Var for the distance measurement.
long tmp_dist;

int min_distance = 2;
int max_distance = 100;

// Filter.
FilterOnePole lpf(LOWPASS, 5);

// Smoothing.
double prev_distance = 0;
int smoothedVal = 0;

// PID.
double input = 0;

void setup() {
  // Start serial communication.
  Serial.begin(9600);

  // Define as in or output.
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
}

void loop() {
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

  tmp_dist = distance;

  if (distance > max_distance) {
    distance = max_distance;
  }
  if (distance < min_distance) {
    distance = min_distance;
  }

  /*if(distance - prev_distance > 20){
    distance = prev_distance; 
  }

  prev_distance = distance;*/
 


  /*distance = (prev_distance + distance) / 2;
  prev_distance = distance;*/

  lpf.input(distance);

  input = map(distance, 2, 70, 0, 100); // 2cm - 50cm => val 0 to 100.

  delay(100);

  

  Serial.println();
  Serial.print("Distance: " + String(tmp_dist));
  Serial.print(" ");
  Serial.print("New distance: " + String(distance));
  Serial.print(" ");
  
}
