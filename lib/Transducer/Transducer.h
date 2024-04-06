#ifndef Transducer_h
#define Transducer_h
#include <Arduino.h>
#include <stdlib.h>

class Transducer {
    private:
        int _pin;
        u_int16_t _minVolt;
        u_int16_t _maxVolt;
        double _minCurrent = .004;
        double _maxCurrent = .02;
        
    public:
        Transducer(int pin);
        u_int16_t get_PSI();
};
#endif

