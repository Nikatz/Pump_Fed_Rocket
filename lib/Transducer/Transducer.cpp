#include <stdlib.h>
#include <Arduino.h>
#include "Transducer.h"
#include <../pins.h>
#include <driver/adc.h>
#include <Filters/Butterworth.hpp>

/**
 * @brief Constructor for the Transducer class
*/
/*Transducer::Transducer(adc1_channel_t value, float slope, float offset) {

    //auto pin = value;

    _x1 = slope;
    _y1 = offset;
    //_minVolt = ((_minVolt_Ducer*_Resistor) / 3.3) * 65536;
    //_maxVolt = ((_maxVolt_Ducer*_Resistor) / 3.3) * 65536; 

 //Setup adc  
    adc1_config_width(ADC_WIDTH_BIT_13);
    adc1_config_channel_atten(TRANSDUCER1_PIN, ADC_ATTEN_DB_11);

  
}

/**
 * @brief Returns the PSI of the transducer
 * @return PSI of the transducer from 0 to 1000
*/
/*//setting up filters
const double f_s = 100; // Hz
const double f_c = 4; // Hz
const double f_n = 2 * f_c / f_s;

auto ducer1_filter = butter<2>(f_n);

float Transducer::get_PSI(){
    float raw = ducer1_filter(adc1_get_raw(TRANSDUCER1_PIN));
    float pressure;
    
    //Only For Calibration 
    //Serial.print(raw);
    //Serial.print(" ");
    //Serial.println(pressure);
    //delay(100);

   return pressure = _x1*raw + _y1;



//old remnants of a dark past

    //_pin = pin;
    //pinMode(_pin, INPUT);
    //Serial.print("minVolt: ");
    //Serial.print(_minVolt);
    //Serial.print("   maxVolt: ");
    //Serial.print(_maxVolt);
    //return map(raw, _minVolt,_maxVolt,minPSI,maxPSI)
}
*/