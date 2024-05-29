#include "DigitalOutput.h"
#include <Arduino.h>


/**
 * @brief Constructor for DigitalOutput class
 * @param pin_number - pin that component is plugged into on the board
 * @return DigitalOutput object
*/
DigitalOutput::DigitalOutput(int pin_number) {
    _pin = pin_number;
    _IO_state = 0;
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
}

/**
 * @brief Returns the state of the I/O pin (HIGH or LOW).
*/
bool DigitalOutput::get_state() {return _IO_state; }

/**
 * @brief Turn on (set to HIGH) the output pin.
 */
void DigitalOutput::turn_on() {
    if (_IO_state) return;
    _IO_state = 1;
    // Serial.printf("turned on %d\n", _pin);
    digitalWrite(_pin, HIGH);
    return;
}


/**
 * @brief Turn off (set to LOW) the output pin.
 */
void DigitalOutput::turn_off() {
    if (!_IO_state) return;
    _IO_state = 0;
    digitalWrite(_pin, _IO_state);
    return;
}

/**
 * @brief Toggle the state of the I/O pin (HIGH to LOW or LOW to HIGH).
 */
void DigitalOutput::toggle() {
    _IO_state = !_IO_state;
    digitalWrite(_pin, _IO_state);
    return;
}
