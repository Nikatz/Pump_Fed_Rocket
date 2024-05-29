#ifndef Transducer_h
#define Transducer_h
#include <Arduino.h>
#include <stdlib.h>
#include <Filters.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <driver/adc.h>

double _x[8] = {1.15, 1.15, 1.15, 1.15, 1.15, 1.15, 1.15, 1.15}; 
double _y[8] = {-1499, -1555 ,-278.32, -278.32, -278.32, -278.32, -278.32, -278.32};

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

int raw1;
int raw2;
int raw3;
int raw4;
int raw5;
int raw6;
int raw7;
int raw8;

#endif

