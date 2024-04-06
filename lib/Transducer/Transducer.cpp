#include <stdlib.h>
#include <Arduino.h>
#include "Transducer.h"


/**
 * @brief Constructor for the Transducer class
*/
Transducer::Transducer(int pin) {
   _pin = pin;
    pinMode(_pin, INPUT);
    _minVolt = (_minVolt_Ducer/ 3.3) * 1024;
    _maxVolt = (_maxVolt_Ducer/ 3.3) * 1024; 
}

/**
 * @brief Returns the PSI of the transducer
 * @return PSI of the transducer from 0 to 1000
*/
u_int16_t Transducer::get_PSI(){
    int raw = analogRead(_pin);
    if (raw < _minVolt) {
        _minVolt = raw;
    }
    if (raw > _maxVolt) {
        _maxVolt = raw;
    }
    return map(raw, _minVolt, _maxVolt, 0, 1000);
}
