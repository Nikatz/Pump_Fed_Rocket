#include <Arduino.h>
#include "Transducer.h"
#include <stdlib.h>
#include <ESP32Servo.h>
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


/*
// MOTOR TIMER:
// Creating a timer for stepping the motor throttle
hw_timer_t *motorStepTimer = NULL;
volatile unsigned long motorTimerCount = 0; // Variable to count milliseconds

// Function to increment motor timer
void IRAM_ATTR onTimer() {
    motorTimerCount++; // Increment the timer count every alarm. Alarm frequency is defined in setup
}

// Create the motor and data pin
const int PULSE_SENDER = THERMISTOR1_PIN; // init motor data pin
Servo Oxy_Motor; // init motor

//power settings to calibrate the ESC
//to calibrate the ESC the full throtle must be held until accepted(5s), then full brake(4s), and then neutral(4s + 5s arming time)
int calibrate[] = {2000, 1000, 1500};
//const int time_RPM = 10000;
const int time_calibrate = 4000;
bool ox_motor_calibrated = false;

void oxy_calibrate(){
    delay(3000);
    
    Oxy_Motor.writeMicroseconds(calibrate[0]);
    delay(5000);

    Oxy_Motor.writeMicroseconds(calibrate[1]);
    delay(time_calibrate);

    Oxy_Motor.writeMicroseconds(calibrate[2]);
    delay(time_calibrate + 1000);

    ox_motor_calibrated = true;
}
*/
/*
//power step for running our system (time pulse width in microseconds)  
//Motor Control 2000 is full throtle 1500 is neutral  
int step[] = {1525, 1550};
int stepTime = 5000;
// currentStep and motorOn are used in loop for motor control logic
int currentStep; // initialize currentStep
bool motorOn; // initialize motorOn. Used to represent if the motor has been turned on. Stays on until turned off by an serial monitor input.

// nextStep is used to iterate through the elements in the step[] array.
void nextStep() { // nextStep takes in an int for the currentStep and return an int for the incremented (if applicable) current step
    if((currentStep+1) < sizeof(step)/sizeof(step[0])) { // if current step is less than amount of elements in step
        currentStep++; // increment step
        Oxy_Motor.writeMicroseconds(step[currentStep]); // set motor to next pwm speed
    } else { // once we go through all the elements in the step array the following lines run
    Oxy_Motor.writeMicroseconds(1500); // turn off the motor by setting it to neutral throttle: 1500 micro seconds
    timerAlarmDisable(motorStepTimer); // stops timer alarms. prevents ticks from turning into alarms.
    motorTimerCount = 0; // resets to first element in step array.
    }
}
*/


/*
//Variable setup for flowmeter counter
volatile unsigned long pulseCount = 0;  // Variable to store the pulse count
volatile unsigned long lastPulseTime = 0;  // Time of the last pulse
float flowRate = 0.0;  // Variable to store the flow rate (in liters per second)
const float pulsesPerLiter = 450.0;  // Example value, replace with your sensor's actual specification
const unsigned long timeoutPeriod = 1000000;  // 0.01 seconds timeout in microseconds (10,000 microseconds)
unsigned long currentTime = 0;

void countPulse() {
  pulseCount++;  // Increment the pulse count
  currentTime = micros();  // Get the current time in microseconds
  // Calculate time between pulses in seconds
  if (lastPulseTime != 0) {  // Ignore the first pulse since there's no previous pulse to compare
    float timeBetweenPulses = (currentTime - lastPulseTime) / 1000000.0;  // Convert microseconds to seconds
    // Calculate the instantaneous flow rate in liters per second
    flowRate = (1.0 / pulsesPerLiter) / timeBetweenPulses;
  }
  // Update the last pulse time to the current time
  lastPulseTime = currentTime;
}
*/


// Initializes the valves
DigitalOutput valve1(VALVE1_PIN);
DigitalOutput valve2(VALVE2_PIN);
DigitalOutput valve3(VALVE3_PIN);
DigitalOutput valve4(VALVE4_PIN);
DigitalOutput valve5(FUEL_VALUE_PIN);
DigitalOutput valve6(OXYGEN_VALVE_PIN);

DigitalOutput Estop_Control(ESTOP_ENABLE_PIN);

// * Control Signals
uint16_t comms = 0x0000;
const u_int16_t bit_valve_1 = 1;
const u_int16_t bit_valve_2 = 1 << 1;
const u_int16_t bit_valve_3 = 1 << 2;
const u_int16_t bit_valve_4 = 1 << 3;
const u_int16_t bit_valve_5 = 1 << 4;
const u_int16_t bit_valve_6 = 1 << 5; 
const u_int16_t bit_ox_motor = 1 << 6;
const u_int16_t bit_cal_ox_motor = 1 << 7;



// * State Machine
enum SystemState {
	STATE_POWERON,
	STATE_FAIL,
	STATE_TEST
};
SystemState STATE;
bool first_time_in_state = true;

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
float pressure[8];

//setting up filter
const double f_s = 100; // Hz
const double f_c = 4; // Hz
const double f_n = 2 * f_c / f_s;


const float MIN_CURRENT = 4e-3;
const float MAX_CURRENT = 20e-3;
const uint16_t RES_VALVE = 150;
const float correcting_factor = 1.31;       // correcting factor for the shitty ADC onboard the ESP32
uint16_t minVolt_420 = (uint16_t)(MIN_CURRENT * RES_VALVE / 3.3 * 8192 * correcting_factor);
uint16_t maxVolt_420 = constrain((uint16_t)(MAX_CURRENT * RES_VALVE / 3.3 * 8192 * correcting_factor), 0, 0x1fff);

// upper and lower bounds for sensors cuz of shitty ADC
const int MIN_PSI = 0;
const int MAX_PSI = 833; 
const int MIN_THRUST_LBS = 0;
const int MAX_THRUST_LBS = 92;

auto ducer1_filter = butter<2>(f_n);
auto ducer2_filter = butter<2>(f_n);
auto ducer3_filter = butter<2>(f_n);
auto ducer4_filter = butter<2>(f_n);
auto ducer5_filter = butter<2>(f_n);
auto ducer6_filter = butter<2>(f_n);
auto ducer7_filter = butter<2>(f_n);
auto ducer8_filter = butter<2>(f_n);

void get_PSI(){


    //Only For Calibration 

	// int psi = 100;
    // Serial.print(psi); 

    ducer1_value = constrain(adc1_get_raw(TRANSDUCER1_PIN), minVolt_420, maxVolt_420);
    pressure[0] = map(ducer1_filter(ducer1_value), minVolt_420, maxVolt_420, MIN_PSI, MAX_PSI);
    ducer2_value = constrain(adc1_get_raw(TRANSDUCER2_PIN), minVolt_420, maxVolt_420);
    pressure[1] = map(ducer2_filter(ducer2_value), minVolt_420, maxVolt_420, MIN_PSI, MAX_PSI);
    ducer3_value = constrain(adc1_get_raw(TRANSDUCER3_PIN), minVolt_420, maxVolt_420);
    pressure[2] = map(ducer3_filter(ducer3_value), minVolt_420, maxVolt_420, MIN_PSI, MAX_PSI);
    ducer4_value = constrain(adc1_get_raw(TRANSDUCER4_PIN), minVolt_420, maxVolt_420);
    pressure[3] = map(ducer4_filter(ducer4_value), minVolt_420, maxVolt_420, MIN_PSI, MAX_PSI);
    adc2_get_raw(TRANSDUCER5_PIN, ADC_WIDTH_BIT_13, &ducer5_value);
    ducer5_value = constrain(ducer5_value, minVolt_420, maxVolt_420);
    pressure[4] = map(ducer5_filter(ducer5_value), minVolt_420, maxVolt_420, MIN_PSI, MAX_PSI);
    adc2_get_raw(TRANSDUCER6_PIN, ADC_WIDTH_BIT_13, &ducer6_value);
    ducer6_value = constrain(ducer6_value, minVolt_420, maxVolt_420);
    pressure[5] = map(ducer6_filter(ducer6_value), minVolt_420, maxVolt_420, MIN_PSI, MAX_PSI);
    adc2_get_raw(TRANSDUCER7_PIN, ADC_WIDTH_BIT_13, &ducer7_value);
    ducer7_value = constrain(ducer7_value, minVolt_420, maxVolt_420);
    pressure[6] = map(ducer7_filter(ducer7_value), minVolt_420, maxVolt_420, MIN_PSI, MAX_PSI);
    adc2_get_raw(TRANSDUCER8_PIN, ADC_WIDTH_BIT_13, &ducer8_value);
    ducer8_value = constrain(ducer8_value, minVolt_420, maxVolt_420);
    pressure[7] = map(ducer8_filter(ducer8_value), minVolt_420, maxVolt_420, MIN_PSI, MAX_PSI);

	 //Serial.print(", ");
	Serial.print("PSI: " );
    Serial.print(pressure[0]);
        Serial.print(",");
    Serial.print(pressure[1]);
        Serial.print(",");
    Serial.print(pressure[2]);
       Serial.print(",");
    Serial.print(pressure[3]);
       Serial.print(",");
    Serial.print(pressure[4]);
       Serial.print(",");
    Serial.print(pressure[5]);
       Serial.print(",");
    Serial.print(pressure[6]);
       Serial.print(",");
    Serial.println(pressure[7]);    
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
    /*if(comms & bit_valve_6){
		valve6.turn_on();
		Serial.println("Valve6: Open");
	}else{
		valve6.turn_off();
		Serial.println("Valve6: Closed");
	}*/
/*    if(comms & bit_ox_motor){
        if(!motorOn){
        motorTimerCount = 0; // ensures the timer is reset to  0
        timerAlarmEnable(motorStepTimer); // Enables the timer alarm (STARTS THE TIMER)
        currentStep = 0; // represents the first item in the step array
        Oxy_Motor.writeMicroseconds(step[currentStep]); // starts the motor with the first item in the step array
        motorOn = true; // data representation of motor on. prevents if statement from running with every loop
		//Serial.println("Motor On!!! VROOOM!!");
        }
	} else if(!comms | !bit_ox_motor){
		Oxy_Motor.writeMicroseconds(1500); // sets the motor to neutral (OFF)
        motorOn = false; // data representation of motor off. allows motor to be turned on again.
        timerAlarmDisable(motorStepTimer); // stops timer alarms. prevents ticks from turning into alarms.
        motorTimerCount = 0; // resets to first element in step array.
        currentStep = 0;
		Serial.println("Motor Off!!!");
	}
    if(comms & bit_cal_ox_motor){
        if(!ox_motor_calibrated){
            oxy_calibrate();
            comms = comms & ~bit_cal_ox_motor;
        }
	}
*/    
}

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
  
    //Estop check setup
    pinMode(ESTOP_SENSE, INPUT);
    Estop_Control.turn_on();

    //setting up the ESC pins
   // Oxy_Motor.attach(PULSE_SENDER);
   // Oxy_Motor.writeMicroseconds(1500);
    //motor calibration setup and timing 
    
    //Ducer setup
    ADC_setup();
    
    //Flow meter set up
   // attachInterrupt(digitalPinToInterrupt(THERMISTOR3_PIN), countPulse, RISING);
/*
    // Starting the motor step timer
    motorStepTimer = timerBegin(0, 80, true); // Initializes timer 0 with a prescaler of 80
    // timer runs every CPU clock. CPU clock speed 80 MHz. Therefore, 80 MHZ / 80 prescale = 1 micro second / tick
    timerAttachInterrupt(motorStepTimer, &onTimer, true); // Attaches the interrupt handler
    timerAlarmWrite(motorStepTimer, 1000, true); // Configures the timer to trigger every 1 ms
    // timerAlarmWrite creates a timer alarm every 1000 ticks. 1000 ticks * 1 micro second / tick = 1 alarm / milli second
    //timerAlarmEnable(motorStepTimer);
    //timerAlarmDisable(motorStepTimer);
*/

} 

void loop() {

    //Serial.println("Current step is: " + String(currentStep));

   /* if (!(digitalRead(ESTOP_SENSE))) {
	 	first_time_in_state = true;
	 	STATE = STATE_FAIL;
	}*/
 
	if (Serial.available() > 0) { // Commands without control box
	 	Serial.print("Input: ");
        comms = Serial.parseInt();
	 	Serial.println(comms, BIN);
	}

    //FlowMeter Pulse Counting 
    // If no pulse is detected for more than the timeout period, set flow rate to zero
    /*if ((currentTime - lastPulseTime) > timeoutPeriod) {
        flowRate = 0.0;  // No pulses received in the timeout period, set flow rate to 0
    }
    */
    // Display the instantaneous flow rate in liters per second
    //Serial.print("Flow Rate (L/s): ");
    //Serial.println(flowRate, 6);  // Display with 6 decimal places for precision

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

    /*for (byte i = 0; i < sizeof(step)/sizeof(step[0]); i++){
    
        Oxy_Motor.writeMicroseconds(step[i]);
        delay(time_RPM);
    }
    */

/*
    // motor timer
    //Serial.println("timer is: " + String(motorTimerCount));
    if(motorTimerCount >= stepTime){ // if motorTimerCount is greater than or equal to defined number of milli seconds
        //Serial.print("Entered >5s if");
        //delay(2000);
        motorTimerCount = 0; // reset time to 0
        nextStep();
    }
*/
    delay(1000);  
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