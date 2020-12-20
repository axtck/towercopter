#include <Servo.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
#define trigPin 10
#define echoPin 9
#define button1 A0
#define button2 A1
#define button3 A2
#define button4 A3
int bt1, bt2, bt3, bt4;
char inChar;
int Tuning;
Servo brushless; 

//----variables sensor
long distance;
long duration;
const int nbaca = 20;
int baca[nbaca];
int index = 0;
int total = 0;
int media = 0;

//-----variables PID
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

//-----constanta motor brushless
const int start = 1000;
const int MIN = 1091;
const int MAX = 1800;

void setup(){
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("Distance=");
  lcd.setCursor(0,1);
  lcd.print("setpoint=");
  brushless.attach(6, MIN, MAX);
  brushless.writeMicroseconds(0);
//  delay(3000);
//  brushless.writeMicroseconds(start);
//  delay(3000);
}
  
void loop(){
  digitalWrite(trigPin ,LOW); 
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin,  LOW);
  bt1 = digitalRead(button1);
  bt2 = digitalRead(button2);
  bt3 = digitalRead(button3);
  bt4 = digitalRead(button4);
  duration = pulseIn(echoPin, HIGH);
  distance = 0.01721*duration;
  total = total - baca[index];
  baca[index] = distance;
  total = total + baca[index];
  index = index+1;
  if(index >= nbaca){
    index = 0;
  }
  media = total/nbaca;
  input = map(media, 2 , 50, 0, 100);
  pid();
  mv = map(out, 0, 100, 1650, 1755);
  brushless.writeMicroseconds(mv);
  delay(10);
  Serial.print(setpoint);
  Serial.print("\t");
  Serial.print(media);
  Serial.print("\t");
  Serial.print(input);
  Serial.print("\t");
  Serial.print(out);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
  Serial.println(mv);
//--------------------------------------------
  lcd.setCursor(11,0);
  lcd.print(media);
  lcd.print(" cm");
  delay(50);
  if(bt1 == 0){
    setpoint = 25;
    lcd.setCursor(9,1);
    lcd.print(setpoint);
    lcd.print("%");
  }
  if(bt2 == 0){
    setpoint = 50;
    lcd.setCursor(9,1);
    lcd.print(setpoint);
    lcd.print("%");
  }
  if(bt3 == 0){
    setpoint = 75;
    lcd.setCursor(9,1);
    lcd.print(setpoint);
    lcd.print("%");
  }
   if(bt4 == 0){
    setpoint = 100;
    lcd.setCursor(9,1);
    lcd.print(setpoint);
    lcd.print("%");
  }
//   if(Tuning == 1){
//    Serial.print(" || ");
//    Serial.print(" Kp :  ");
//    Serial.print(kp);
//    Serial.print(" Ki : ");
//    Serial.print(ki);
//    Serial.print(" Kd : ");
//    Serial.print(kd);
//  }
}

//void serialEvent(){
//  while(Serial.available()){
//    inChar = Serial.read();
//    //PILIH MODE TUNING
//    if(inChar == 'z'){Tuning = 1;}
//    if(inChar == 'x'){Tuning = 0;}
//
//      //TUNING NILAI KP
//      if (inChar == 'q' && Tuning == 1){kp+= 0.01;} // kp = kp + 0.1
//      if (inChar == 'a' && Tuning == 1){kp-= 0.01;}
//      if (inChar == 'w' && Tuning == 1){kp+= 0.001;}
//      if (inChar == 's' && Tuning == 1){kp-= 0.001;}
//      //TUNING NILAI KI
//      if (inChar == 'r' && Tuning == 1){ki+= 0.01;}
//      if (inChar == 'f' && Tuning == 1){ki-= 0.01;}
//      if (inChar == 't' && Tuning == 1){ki+= 0.001;}
//      if (inChar == 'g' && Tuning == 1){ki-= 0.001;}
//      //TUNING NILAI Kd
//      if (inChar == 'u' && Tuning == 1){kd+= 0.01;}
//      if (inChar == 'j' && Tuning == 1){kd-= 0.01;}
//      if (inChar == 'i' && Tuning == 1){kd+= 0.001;}
//      if (inChar == 'k' && Tuning == 1){kd-= 0.001;}
//  }
//}
