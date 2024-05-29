#include <Arduino.h>
#include <../pins.h>
#include "ValvesControl.h"

ValveControl::ValveControl(int pin){
    _pin = pin;
    _IO_state = 0;

    pinMode(_pin, INPUT);
    analogWrite(_pin,0);
}

bool  ValveControl::get_state() {return _IO_state; }

void ValveControl::open() {
    Serial.println("High V");

    digitalWrite(_pin, HIGH);
    
}

void ValveControl::close(){
    
    Serial.println("Low V");
    digitalWrite(_pin, LOW);

}