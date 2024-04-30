#ifndef Transducer_h
#define Transducer_h
#include <Arduino.h>
#include <stdlib.h>
#include <Filters.h>
#include <AH/Timing/MillisMicrosTimer.hpp>

class Transducer {
    private:
        int _pin;
        float _Resistor = 149.85;
        float _x;
        float _y;
    public:
        Transducer(int pin, float slope, float offset);
        float get_PSI();
};
#endif

