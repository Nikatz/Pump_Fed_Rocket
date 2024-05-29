#include <stdlib.h>
#include <Arduino.h>
#include "Transducer.h"
#include <../pins.h>
#include <driver/adc.h>
#include <Filters/Butterworth.hpp>

/**
 * @brief Constructor for the Transducer class
*/


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

/**
 * @brief Returns the PSI of the transducer
 * @return PSI of the transducer from 0 to 1000
*/
//setting up filter
const double f_s = 100; // Hz
const double f_c = 4; // Hz
const double f_n = 2 * f_c / f_s;

auto ducer_filter = butter<2>(f_n);

void get_PSI(){


    //Only For Calibration 
    //Serial.print(raw);
    //Serial.print(" ");
    //Serial.println(pressure);
    //delay(100);

    raw1 = ducer_filter(adc1_get_raw(TRANSDUCER1_PIN));
        pressure1 = _x[0]*raw1 + _y[0];
    raw2 = ducer_filter(adc1_get_raw(TRANSDUCER2_PIN));
        pressure2 = _x[1]*raw2 + _y[1];
    raw3 = ducer_filter(adc1_get_raw(TRANSDUCER3_PIN));
        pressure3 = _x[2]*raw3 + _y[2];
    raw4 = ducer_filter(adc1_get_raw(TRANSDUCER4_PIN));
        pressure4 = _x[3]*raw4 + _y[3];
    adc2_get_raw(TRANSDUCER5_PIN, ADC_WIDTH_BIT_13, &raw5);
        raw5 = ducer_filter(raw5);
        pressure5 = _x[4]*raw5 + _y[4];
    adc2_get_raw(TRANSDUCER5_PIN, ADC_WIDTH_BIT_13, &raw6);
        raw6 = ducer_filter(raw6);
        pressure6 = _x[5]*raw6 + _y[5];
    adc2_get_raw(TRANSDUCER7_PIN, ADC_WIDTH_BIT_13, &raw7);
        raw7 = ducer_filter(raw7);
        pressure7 = _x[6]*raw7 + _y[6];
    adc2_get_raw(TRANSDUCER8_PIN, ADC_WIDTH_BIT_13, &raw7);
        pressure8 = _x[7]*raw8 + _y[7];



    Serial.print("PSI: " );
    Serial.print(pressure1);
        Serial.print(",");
    Serial.print(pressure2);
        Serial.print(",");
    Serial.print(pressure3);
        Serial.print(",");
    Serial.print(pressure4);
        Serial.print(",");
    Serial.print(pressure5);
        Serial.print(",");
    Serial.print(pressure6);
        Serial.print(",");
    Serial.print(pressure7);
        Serial.print(",");
    Serial.print(pressure8);    
}

//old remnants of a dark past

    //_pin = pin;
    //pinMode(_pin, INPUT);
    //Serial.print("minVolt: ");
    //Serial.print(_minVolt);
    //Serial.print("   maxVolt: ");
    //Serial.print(_maxVolt);
    //return map(raw, _minVolt,_maxVolt,minPSI,maxPSI)

/*Transducer::Transducer(adc1_channel_t value, float slope, float offset) {

    auto pin = value;

    _x1 = slope;
    _y1 = offset;
    //_minVolt = ((_minVolt_Ducer*_Resistor) / 3.3) * 65536;
    //_maxVolt = ((_maxVolt_Ducer*_Resistor) / 3.3) * 65536; 

}*/