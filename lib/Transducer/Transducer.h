#ifndef Transducer_h
#define Transducer_h
#include <Arduino.h>
#include <stdlib.h>
#include <Filters.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <driver/adc.h>

double _x[8] = {1.15, 1.15, 1.15, 1.15, 1.15, 1.15, 1.15, 1.15}; 
double _y[8] = {-278, -278.32 ,-278.32, -278.32, -278.32, -278.32, -278.32, -278.32};

void get_PSI();
void ADC_setup();

double pressure1;
double pressure2;
double pressure3;
double pressure4;
double pressure5;
double pressure6;
double pressure7;
double pressure8;

int ducer1_value;
int ducer2_value;
int ducer3_value;
int ducer4_value;
int ducer5_value;
int ducer6_value;
int ducer7_value;
int ducer8_value;

#endif

