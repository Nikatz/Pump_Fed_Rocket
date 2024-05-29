#ifndef ValvesControl_h
#define ValvesControl_h
#include <Arduino.h>
#include <../pins.h>

class ValveControl {
    private:
        int _pin;
        bool _IO_state;

    public:
        ValveControl(int pin);
        bool get_state();
        void open();
        void close();
        
};

#endif