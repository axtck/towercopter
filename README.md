# Tower copter

***System PID control on a tower copter***

---
## Idea

Realizing a system to drive a propeller attached to a motor, using an ultrasonic sensor to provide feedback. 

I would first like to try this using an Arduino, then maybe switch to a python script that runs on a Raspberry Pi.

The brushless motor is driven by an ESC (electronic speed controller) controlled by an Arduino or RPi. When the platform ascends in the frame, the ultrasonic sensor will return the distance values that will be used as feedback for the system.

The goal is to have a balancing platform, based on PID control.

---

## Componenets

* controller (Arduino, RPi)
* brushless motor (DYS D2830, 750kv)
* brushless ESC (electronic speed controller) (30A)
* propeller (12cm)
* ultrasonic sensor (HC-SR04)
* power supply (12V-16V 30A)
* wires
* frame

---

## TODO

#### Hardware
* [X] do research
* [X] order components
* [X] wait for components
* [X] build frame
* [X] design platform that fits in frame (CAD)
* [X] print design
* [X] setup controllers (fresh Arduino & RPi)
* [X] build complete setup


#### Software
* [X] research Arduino libraries
* [X] write control for Arduino
* [X] write complete Arduino code
* [ ] research python libraries
* [ ] write HC-SR04 code for RPi
* [ ] write control for RPi (python)
* [ ] write complete RPi code
