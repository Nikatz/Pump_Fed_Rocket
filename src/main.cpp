#include <Arduino.h>
#include "Transducer.h"
#include <stdlib.h>
#include <ESP32Servo.h>

/*
Code By Nick Katz and help from Danny Gutierrez
Writen to control a Electronic Speed Contoler and a Motor
*/
/************************************************
 * HERE STARTS THE REAL CODE           
 *  /\/\/\                            /  \
 * | \  / |                         /      \
 * |  \/  |                       /          \
 * |  /\  |----------------------|     /\     |
 * | /  \ |                      |    /  \    |
 * |/    \|                      |   /    \   |
 * |\    /|                      |  | (  ) |  |
 * | \  / |                      |  | (  ) |  |
 * |  \/  |                 /\   |  |      |  |   /\
 * |  /\  |                /  \  |  |      |  |  /  \
 * | /  \ |               |----| |  |      |  | |----|
 * |/    \|---------------|    | | /|   .  |\ | |    |
 * |\    /|               |    | /  |   .  |  \ |    |
 * | \  / |               |    /    |   .  |    \    |
 * |  \/  |               |  /      |   .  |      \  |
 * |  /\  |---------------|/        |   .  |        \|
 * | /  \ |              /   SPT    |   .  |  SPT     \
 * |/    \|              (          |      |           )
 * |/\/\/\|               |    | |--|      |--| |    |
 * ------------------------/  \-----/  \/  \-----/  \--------
 *                         \\//     \\//\\//     \\//
 *                          \/       \/  \/       \/
 ************************************************
*/

//TODO RPM Control 
//TODO Pressure Data Accuracy 
//TODO Tempurature Data
//TODO DAC Display
//TODO Solenoid control

Transducer ducer_1 (17,0.140546760109178,-250.707921137022);
const int PULSE_SENDER = 10;

//SETS THE MOTOR
//const int PulseSender = 10;
Servo speedControler;

//power step for running our system (time pulse width in microseconds)  
//Motor Control 2000 is full throtle 1500 is neutral  
int step[] = {1600, 1625, 1750, 1775};

//power settings to calibrate the ESC
//to calibrate the ESC the full throtle must be held until accepted(5s), then full brake(4s), and then neutral(4s + 5s arming time)
int calibrate[] = {2000, 1000, 1500};
const int time_RPM = 10000;
const int time_calibrate = 4000;


void setup() {
  Serial.begin(9600);
  
/*
  //setting up the ESC pins
  speedControler.attach(PULSE_SENDER);

  // Calibrates the ESC
  speedControler.writeMicroseconds(calibrate[0]);
  delay(5000);

  speedControler.writeMicroseconds(calibrate[1]);
  delay(time_calibrate);

  speedControler.writeMicroseconds(calibrate[2]);
  delay(time_calibrate + 10000);

 
 /* for (byte i = 0; i < sizeof(step)/sizeof(step[0]); i++){
    
    speedControler.writeMicroseconds(step[i]);
    delay(time_RPM);
  }
  
  

  //Turns off the motor
  speedControler.writeMicroseconds(1500);*/

  
} 

void loop() {
float psi = ducer_1.get_PSI();

Serial.print("PSI: ");
Serial.println(psi);
//Serial.print("PSIG: ");
//Serial.println(psi);
//delay(1000);
}