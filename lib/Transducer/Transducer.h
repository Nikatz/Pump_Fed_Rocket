#ifndef Transducer_h
#define Transducer_h
#include <Arduino.h>
#include <stdlib.h>

class Transducer {
    private:
        int _pin;
        float _minVolt_Ducer= 4e-3;
        float _maxVolt_Ducer = 20e-3;
        float _minVolt = 0;
        float _maxVolt = 0;
        float _Resistor = 150;
    public:
        Transducer(int pin);
        u_int16_t get_PSI();
};
#endif

