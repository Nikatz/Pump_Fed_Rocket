#include <stdlib.h>
#include <Arduino.h>
#include "Transducer.h"
#include <driver/adc.h>
#include <Filters/Butterworth.hpp>

/**
 * @brief Constructor for the Transducer class
*/
Transducer::Transducer(int pin, float slope, float offset) {
    //_pin = pin;
    //pinMode(_pin, INPUT);

    _x = slope;
    _y = offset;
    //_minVolt = ((_minVolt_Ducer*_Resistor) / 3.3) * 65536;
    //_maxVolt = ((_maxVolt_Ducer*_Resistor) / 3.3) * 65536; 

    adc1_config_width(ADC_WIDTH_BIT_13);
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);

//Setup adc    


}

void setup_ADC() {

}

/**
 * @brief Returns the PSI of the transducer
 * @return PSI of the transducer from 0 to 1000
*/
//setting up filters
const double f_s = 100; // Hz
const double f_c = 4; // Hz
const double f_n = 2 * f_c / f_s;

auto ducer_filter = butter<2>(f_n);

float Transducer::get_PSI(){
    float raw = constrain (adc1_get_raw(_pin), 11916, 59578);
    //int raw = 10000; 
    float pressure;
    //Only For Calibration 
   
    Serial.print(raw);
    Serial.print(" ");
    //Serial.println(pressure);
     delay(100);

    return pressure = _x*raw + _y;

//old remnants of a dark past
    //Serial.print("minVolt: ");
    //Serial.print(_minVolt);
    //Serial.print("   maxVolt: ");
    //Serial.print(_maxVolt);
    //return map(raw, _minVolt,_maxVolt,minPSI,maxPSI)
}
