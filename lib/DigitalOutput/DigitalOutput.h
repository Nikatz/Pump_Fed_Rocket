#ifndef DigitalOutput_h
#define DigitalOutput_h
#include <Arduino.h>

class DigitalOutput {
    private:
        int _pin;
        bool _IO_state;

    public:
        DigitalOutput(int pin);
        bool get_state();
        void turn_on();
        void turn_off();
        void toggle();
};
#endif