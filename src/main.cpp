#include <Arduino.h>
#include "Transducer.h"
#include <stdlib.h>
//#include <ESP32Servo.h>
#include "../Pins.h" 
#include <driver/adc.h>
#include <Filters/Butterworth.hpp>
#include "ValvesControl.h"
#include "DigitalOutput.h"



/*
Code By Nick Katz, Luke Yium, and help from Danny Gutierrez Based on MK1_Better
Writen To Control The Pump Fed Rocket Engine's valves and motors And Collect Data 
*/
/************************************************
 *                                     /\ 
 *                                    /  \
 *                                   /    \ 
 *  /\/\/\                          /      \
 * | \  / |                        /        \
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

//TODO Tempurature Data
//TODO DAC Display
//TODO Solenoid control - maybe done need to test


// Initializes the valves
DigitalOutput valve1(VALVE1_PIN);
DigitalOutput valve2(VALVE2_PIN);
DigitalOutput valve3(VALVE3_PIN);
DigitalOutput valve4(VALVE4_PIN);
DigitalOutput valve5(FUEL_VALUE_PIN);
DigitalOutput valve6(OXYGEN_VALVE_PIN);

DigitalOutput Estop_Control(ESTOP_ENABLE_PIN);

// function declarations for ESC controller :
unsigned int readRegister(int);
unsigned int writeRegister(int, unsigned int); 
long getRPM(); 
unsigned int writeThrottle(unsigned int);  
void clearRegister(); 
byte intToByte(int);
void calibrate(); 
void startup(); 
float getPower(); 


// defining the tx and rx pins for ESC control

const int serial1_r = 16;
const int serial1_t = 17;

// defining the throttle registers for ESC control (see https://www.castlecreations.com/en/serial-link-010-0121-00page 5)
const int rpmRegister = 5;
const int voltageRegister = 0; 
const int currentRegister = 2; 
const int throttleRegister = 128;
const int EStopRegister = 130;
int deviceID = 0; //+128;
// throttle numbers for ESC control
const unsigned int throttleNeutral = 32767; 
long rpmCurrent; 
unsigned int throttleCurr; 
unsigned int writeResponse; 
float powerCurr; 

// PID values for ESC control
float error;  // errors in rpm 
float ierror; 
float prevError;  
float controller; 

long setRPM = 40000; 
unsigned long currentTime; 
unsigned long prevTime; 
unsigned long dt; 

float Kp = 0.0003; 
float Ki = 0.0005; 
float Kd = 0.003; 

// * Control Signals
uint16_t comms = 0x0000;
const u_int16_t bit_valve_1 = 1;
const u_int16_t bit_valve_2 = 1 << 1;
const u_int16_t bit_valve_3 = 1 << 2;
const u_int16_t bit_valve_4 = 1 << 3;
const u_int16_t bit_valve_5 = 1 << 4;
const u_int16_t bit_valve_6 = 1 << 5; 
// const u_int8_t bit_valid = 1 << 6;
//const u_int8_t bit_heartbeat = 1 << 7;
 
// * State Machine
enum SystemState {
	STATE_POWERON,
	STATE_FAIL,
	STATE_TEST
};
SystemState STATE;
bool first_time_in_state = true;

// //////////////////////////////////////////////////////////////////////////////////////
// //Motor Contol Funtions
// //////////////////////////////////////////////////////////////////////////////////////

// put function definitions here:

// function reads a register value from the esc 
unsigned int readRegister(int registerNum) {
// to read rpm, need to send a command packet, then read the response,
// then convert the reading to an rpm response
// command packet

    byte commPack[] = {intToByte(deviceID+128), intToByte(registerNum), 0x00, 0x00, 0x00};
    byte checksum = 0x00; 
    for (int i = 0; i < 4; i++) {
        checksum = checksum - commPack[i];
    }
    commPack[4] = checksum; 
    Serial1.write(commPack, sizeof(commPack)); // commPack is good to write 

    // reading the response 
    byte response[3];// = {0x41, 0x42, 0x43}; 
    while(Serial1.available() < 3); 
    for (int jj = 0; jj < 3; jj++) { 
      byte temp = Serial1.read(); 
      //Serial.print(temp); 
      //Serial.print("  "); 
      response[jj] = temp; 
    }

    unsigned int registerVal = (response[0] << 8) | response[1]; 
    return registerVal;
}

// function writes an integer number to a specified register 
unsigned int writeRegister(int registerNum, unsigned int value) {
    // convert value and register to binary, keep the same 
    byte upper = highByte(value); 
    byte lower = lowByte(value); 

    byte commPack[] = {intToByte(deviceID+128), intToByte(registerNum), upper, lower, 0x00};
    byte checksum = 0x00; 
    for (int i = 0; i < 4; i++) {
        checksum = checksum - commPack[i];
    }
    commPack[4] = checksum; 
    Serial1.write(commPack, sizeof(commPack)); // commPack is good to write 

    // reading the response 
    byte response[3]; 
    while(Serial1.available() < 3); 
    for (int jj = 0; jj < 3; jj++) { 
        response[jj] = Serial1.read(); 
    }

    unsigned int registerVal = (response[0] << 8) | response[1]; 
    return registerVal;

}

// This function specifically read the RPM value of the ESC 
long getRPM() {
    // getting register value 
    unsigned int rpm = readRegister(rpmRegister); 
    // scaling 
    rpm = long(rpm); 
    rpm = rpm*208678.6; 
    rpm = rpm/20420; 
    return rpm; 
}

float getPower() {
  // getting register value 
  unsigned int volt = readRegister(voltageRegister); 
  unsigned int curr = readRegister(currentRegister); 

  float vol = volt/2042.0 * 20.0; 
  float cur = curr/2042.0 * 50.0; 

  return vol*cur; 
}

// specifically writes to the throttle register a throttle value 
// Throttle is 0-65535, with 0 corresponding to 1 ms pulse(full reverse) 
// 65535 is a 2 ms pulse (full forward), and 32767 is neutral 
unsigned int writeThrottle(unsigned int throttleVal) {
    unsigned int response = writeRegister(throttleRegister, throttleVal); 
    return response; 
}

// function clears the register to prepare a new command 
// not always necessary but recommended (especially at start of a run) 
void clearRegister() {
    byte clearer = 0x00; 
    if (Serial1.available() == 0) {
        for(int ii = 0; ii < 10; ii++) {
            Serial1.write(clearer); 
        }
    }
}

byte intToByte(int val) {
// to preserve, val must be 255 or less, otherwise truncated
    return byte(val);
}

void calibrate() {
  unsigned int store = writeThrottle(65535); 
  delay(5000); 
  store = writeThrottle(0); 
  delay(4000); 
  store = writeThrottle(throttleNeutral); 
  delay(7000); 
}

// it seems as though the esc needs a few seconds of neutral throttle to 
//  initialize the Serial connection. Function sends neutral for 2 seconds
void startup() {
  for(int ii = 0; ii < 100; ii++) {
    rpmCurrent = getRPM(); 
    clearRegister(); 

    writeResponse = writeThrottle(throttleNeutral);  
    clearRegister(); 

    delay(20); 
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void ADC_setup() {



//Setup adc  
    adc1_config_width(ADC_WIDTH_BIT_13);
    adc1_config_channel_atten(TRANSDUCER1_PIN, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(TRANSDUCER2_PIN, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(TRANSDUCER3_PIN, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(TRANSDUCER4_PIN, ADC_ATTEN_DB_11);
    adc2_config_channel_atten(TRANSDUCER5_PIN, ADC_ATTEN_DB_11);
    adc2_config_channel_atten(TRANSDUCER6_PIN, ADC_ATTEN_DB_11);
    adc2_config_channel_atten(TRANSDUCER7_PIN, ADC_ATTEN_DB_11);
    adc2_config_channel_atten(TRANSDUCER8_PIN, ADC_ATTEN_DB_11);


}

// constants for ducer 
u_int16_t pressure[8];

//setting up filter
const double f_s = 100; // Hz
const double f_c = 4; // Hz
const double f_n = 2 * f_c / f_s;

auto ducer_filter = butter<2>(f_n);

void get_PSI(){


    //Only For Calibration 

	// int psi = 100;
    // Serial.print(psi); 
   // delay(500);

    raw1 = ducer_filter(adc1_get_raw(TRANSDUCER1_PIN));
        pressure[0] = _x[0]*raw1 + _y[0];
    raw2 = ducer_filter(adc1_get_raw(TRANSDUCER2_PIN));
        pressure[1] = constrain(_x[1]*raw2 + _y[1], 0, 1000);
    raw3 = ducer_filter(adc1_get_raw(TRANSDUCER3_PIN));
        pressure[2] = constrain(_x[2]*raw3 + _y[2], 0, 1000);
    raw4 = ducer_filter(adc1_get_raw(TRANSDUCER4_PIN));
        pressure[3] = constrain(_x[3]*raw4 + _y[3], 0, 1000);
    adc2_get_raw(TRANSDUCER5_PIN, ADC_WIDTH_BIT_13, &raw5);
        raw5 = ducer_filter(raw5);
        pressure[4] = constrain(_x[4]*raw5 + _y[4], 0, 1000);
    adc2_get_raw(TRANSDUCER5_PIN, ADC_WIDTH_BIT_13, &raw6);
        raw6 = ducer_filter(raw6);
        pressure[5] = constrain(_x[5]*raw6 + _y[5], 0, 1000);
    adc2_get_raw(TRANSDUCER7_PIN, ADC_WIDTH_BIT_13, &raw7);
        raw7 = ducer_filter(raw7);
        pressure[6] = constrain(_x[6]*raw7 + _y[6], 0, 1000);
    adc2_get_raw(TRANSDUCER8_PIN, ADC_WIDTH_BIT_13, &raw7);
        pressure[7] = constrain(_x[7]*raw8 + _y[7], 0, 1000);

	 //Serial.print(", ");
	 Serial.println(raw1);
	 Serial.print("PSI: " );
      Serial.print(pressure[0]);
        Serial.print(",");
    Serial.print(pressure[1]);
    //    Serial.print(",");
    //Serial.print(pressure[2]);
    //    Serial.print(",");
    //Serial.print(pressure[3]);
    //    Serial.print(",");
    //Serial.print(pressure[4]);
    //    Serial.print(",");
    //Serial.print(pressure[5]);
     //   Serial.print(",");
    //Serial.print(pressure[6]);
      //  Serial.print(",");
    //Serial.println(pressure[7]);    
}

void power_on_state(){
    if (first_time_in_state) {
		//Serial.printf("POWERON entry\n");
		valve1.turn_off();
		valve2.turn_off();
		valve3.turn_off();
		valve4.turn_off();
        valve5.turn_off();
        valve6.turn_off();
		first_time_in_state = false;
	}

	//Serial.println(comms,BIN);

	//If statements to control valves 
	if(comms & bit_valve_1){
		valve1.turn_on();
		Serial.println("Valve1: Open");
		}else{
		valve1.turn_off();
		Serial.println("Valve1: Closed");
	}
		if(comms & bit_valve_2){
		valve2.turn_on();
		Serial.println("Valve2: Open");
		}else{
		valve2.turn_off();
		Serial.println("Valve2: Closed");
	}
		if(comms & bit_valve_3){
		valve3.turn_on();
		Serial.println("Valve3: Open");
		}else{
		valve3.turn_off();
		Serial.println("Valve3: Closed");
	}
		if(comms & bit_valve_4){
		valve4.turn_on();
		Serial.println("Valve4: Open");
		}else{
		valve4.turn_off();
		Serial.println("Valve4: Closed");
	}
		if(comms & bit_valve_5){
		valve5.turn_on();
		Serial.println("Valve5: Open");
	}else{
		valve5.turn_off();
		Serial.println("Valve5: Closed");
	}
		if(comms & bit_valve_6){
		valve6.turn_on();
		Serial.println("Valve6: Open");
	}else{
		valve6.turn_off();
		Serial.println("Valve6: Closed");
	}

// 	//ESC Control
// 	powerCurr = getPower(); 
//   clearRegister(); 

//   rpmCurrent = getRPM(); 
//   currentTime = millis(); 
//   clearRegister(); 

//   error = setRPM - rpmCurrent; //rpm 
//   dt = (currentTime - prevTime); //ms 
//   ierror += error*dt; 
  
//   // casting and checking for unsendable values 
//   controller = Kp*error + Ki*ierror + Kp*(error - prevError)/dt; 
//   //Serial.print(controller); 

//   if (controller < 0) {
//     controller = 0; 
//   } else if (controller > 32767) {
//     controller = 32767; 
//   } 
//   controller = floor(controller + 32767); 
//   throttleCurr = static_cast<unsigned int>(controller); 

//   writeResponse = writeThrottle(throttleCurr); 
//   clearRegister(); 

//   prevTime = currentTime; 
//   prevError = error; 

//   //Serial.print(" rpm: "); 
//   Serial.print(rpmCurrent);  

//   Serial.print(", "); 
//   Serial.print(powerCurr); 

//   Serial.print(", "); 
//   Serial.print(throttleCurr); 

//   Serial.println(); 
//   delay(20); 

};


//TODO Refit logic
void fail_state () {
	if (first_time_in_state) {
		//estop_enable.turn_off();
		// ign_wire.turn_off();
		valve1.turn_off();
		valve5.turn_off();
		valve2.turn_off();
		valve3.turn_off();
		valve4.turn_off();
        valve6.turn_off();

		if(!comms) {
			// green_light.turn_on();
			// yellow_light.turn_off();
			// red_light.turn_on();
            Serial.print("Comms Loss");
		} else {
			// green_light.turn_on();
			// yellow_light.turn_on();
			// red_light.turn_off();
            Serial.print("E-Stop");
		}
	}

	
	// green_light.toggle();
	// yellow_light.toggle();
	// red_light.toggle();
	return;
}

void test_state() {
	return;
}


void setup() {
  Serial.begin(115200);
  
  pinMode(ESTOP_SENSE, INPUT);
  Estop_Control.turn_on();
  ADC_setup();


	// // For ESC control
	// Serial1.begin(115200, SERIAL_8N1, serial1_r, serial1_t);
	//   Serial.println("Serial begin"); 

	//   delay(4000); 
	//   clearRegister(); 

	//   delay(2000); 

	//   Serial.println("startup"); 
	//   startup(); 
	//   Serial.println("rpm, power [W], throttle");   

	//   //delay(3000); 
	//   prevError = setRPM; 
	//   prevTime = millis(); 
	//   delay(20); 
} 

void loop() {



    //  if (!(digitalRead(ESTOP_SENSE))) {
	// 	first_time_in_state = true;
	// 	STATE = STATE_FAIL;
	// }
 
	 if (Serial.available() > 0) { // Commands without control box
	 	Serial.print("Input: ");
		//delay(2000);
        comms = Serial.parseInt();
	 	Serial.println(comms, BIN);
	 }

     if (Serial.available() == 0) { // Commands without control box
	 	//Serial.println("Comms Fail");
        //comms = Serial.read();
	 	Serial.println(comms, BIN);
	 }

	// State machine to determine if were in power on or fail state
	switch (STATE) {
		case STATE_POWERON:
			power_on_state();
            //Serial.println("IN Power on");

			break;
		case STATE_FAIL:
			//fail_state();
            Serial.println("IN fail");
			if (comms == 0x80 && digitalRead(ESTOP_SENSE)) {
				STATE = STATE_POWERON;
				first_time_in_state = true;
			}
			break;
		// Testing new features
		case STATE_TEST: 
			//test_state();
            Serial.println("IN Test");
			break;
		default:
			STATE = STATE_FAIL;
			break;
	}

    get_PSI();


	delay(500);
}

//Ancestor code
//SETS THE MOTOR
/*const int PulseSender = 10;
Servo Oxy_Motor;


//power step for running our system (time pulse width in microseconds)  
//Motor Control 2000 is full throtle 1500 is neutral  
int step[] = {1600, 1625, 1750, 1775};

//power settings to calibrate the ESC
//to calibrate the ESC the full throtle must be held until accepted(5s), then full brake(4s), and then neutral(4s + 5s arming time)
int calibrate[] = {2000, 1000, 1500};
const int time_RPM = 10000;
const int time_calibrate = 4000;*/


/*
  //setting up the ESC pins
  Oxy_Motor.attach(PULSE_SENDER);

  // Calibrates the ESC
  Oxy_Motor.writeMicroseconds(calibrate[0]);
  delay(5000);

  Oxy_Motor.writeMicroseconds(calibrate[1]);
  delay(time_calibrate);

  Oxy_Motor.writeMicroseconds(calibrate[2]);
  delay(time_calibrate + 10000);

 
 /* for (byte i = 0; i < sizeof(step)/sizeof(step[0]); i++){
    
    Oxy_Motor.writeMicroseconds(step[i]);
    delay(time_RPM);
  }
  
  

  //Turns off the motor
  Oxy_Motor.writeMicroseconds(1500);*/

  