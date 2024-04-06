#ifndef Transducer_h
#define Transducer_h
#include <Arduino.h>
#include <stdlib.h>

class Transducer {
    private:
        int _pin;
        u_int16_t _minVolt_Ducer= 0;
        u_int16_t _maxVolt_Ducer = 0.1;
                
    public:
        Transducer(int pin);
        u_int16_t get_PSI();
};
#endif

